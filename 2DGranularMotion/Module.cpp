#include "Module.hpp"

#include "Face.hpp"

#include "Robot.hpp"

Module::Module(sf::Vector2i position, int pressureID, std::shared_ptr<Robot> robot, sf::Texture& texture)
{
	m_pressureID = pressureID;
	m_position = position;
	m_robot = robot;
	m_numFiringFaces = 0;
	m_numFiringFacesInto = 0;
	m_pressure = 0;

	m_CoMRelativePosition = sf::Vector2f(m_position) - robot->GetCoM() + sf::Vector2f(0.5, 0.5);

	for (std::vector<sf::Sprite>::iterator spriteIt = m_pumpSprites.begin(); spriteIt != m_pumpSprites.end(); spriteIt ++)
	{
		sf::IntRect rect;
		rect.left = 65;
		rect.top = 1;
		rect.width = 12;
		rect.height = 12;

		spriteIt->setTexture(texture);
		spriteIt->setTextureRect(rect);
		spriteIt->setOrigin(12, 12);
	}
}

void Module::SetUpFaces(int faceID1, int faceID2, int faceID3, int faceID4)
{

	m_faces.push_back(std::shared_ptr<Face>(new Face(UP, faceID1, !m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(0, -1)), shared_from_this())));
	m_faces.push_back(std::shared_ptr<Face>(new Face(RIGHT, faceID2, !m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(1, 0)), shared_from_this())));
	m_faces.push_back(std::shared_ptr<Face>(new Face(DOWN, faceID3, !m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(0, 1)), shared_from_this())));
	m_faces.push_back(std::shared_ptr<Face>(new Face(LEFT, faceID4, !m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(-1, 0)), shared_from_this())));
}

void Module::Draw(sf::RenderWindow* window)
{
	////8 sprites, 4 for pump running, 4 for water flowing (arrows really)

	//sf::RectangleShape rect;

	//rect.setSize(sf::Vector2f(10, 10));
	//rect.setOrigin(sf::Vector2f(5, 5));

	//short rotation = m_robot.lock()->GetRotation();
	//sf::Vector2f position;

	//position.x = m_robot.lock()->GetPosition().x + m_CoMRelativePosition.x * MODULE_SIZE * cos(float(rotation)) - m_CoMRelativePosition.y * MODULE_SIZE * sin(float(rotation));
	//position.x = m_robot.lock()->GetPosition().x + m_CoMRelativePosition.x * MODULE_SIZE * cos(float(rotation)) - m_CoMRelativePosition.y * MODULE_SIZE * sin(float(rotation));

}

void Module::SelectFace(sf::Vector2f position, int command)
{
	short direction = 4;

	if (position.x > 6 && position.y > 6)
	{
		direction = RIGHT;
	}

	else if (position.x > 6 && position.y < -6)
	{
		direction = UP;
	}

	else if (position.x < -6 && position.y < -6)
	{
		direction = LEFT;
	}

	else if (position.x < -6 && position.y > 6)
	{
		direction = DOWN;
	}



	if (direction != 4)
	{
		if (command == TOGGLE_ACTIVATE)
		{
			m_robot.lock()->SetUpdateFlows();

			if (m_faces[direction]->GetFiring())
			{
				m_faces[direction]->SetNotFiring();

				m_numFiringFaces --;

				if (!m_robot.lock()->IsModuleHere(m_position + m_faces[direction]->GetNormalVector()))
				{
					m_robot.lock()->StopFiringExternalFace();
				}
			}
			else if (!m_faces[direction]->GetBlocked())
			{
				//If trying to turn the pump on, need to perform checks

				//First, are 2 of this module's pumps firing already
				if (m_numFiringFaces < 2)
				{

					//Now check if there's a module in the direction that this pump will fire
					//if (m_robot.lock()->IsModuleHere(m_position + m_faces[direction]->GetNormalVector()))
					if (!m_faces[direction]->GetExternal())
					{
						//If there is, check that the pump of the opposite face of the neighbouring module isn't firing
						if (!m_robot.lock()->GetModule(m_position + m_faces[direction]->GetNormalVector()).lock()->GetFace((direction + 2) % 4).lock()->GetFiring())
						{
							m_faces[direction]->SetFiring();

							m_numFiringFaces ++;
						}
					}

					else
					{
						//If there isn't (the face is external), check if there are already too many external faces firing
						//if (m_robot.lock()->CanFireExternalFace())
						//{
							m_faces[direction]->SetFiring();

							m_robot.lock()->FireExternalFace();

							m_numFiringFaces ++;
						//}
					}
				}
			}
		}

		else if (command == TOGGLE_BLOCK)
		{
			m_robot.lock()->SetUpdateFlows();

			if (m_faces[direction]->GetBlocked())
			{
				m_faces[direction]->SetUnblocked();
			}
			else if (!m_faces[direction]->GetFiring())
			{
				m_faces[direction]->SetBlocked();
			}
		}
	}
}

bool Module::IsAnyFaceFiring()
{
	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if ((*faceIt)->GetFiring())
		{
			return true;
		}
	}

	return false;
}

bool Module::IsFaceFiring(int face)
{
	return m_faces[face]->GetFiring();
}



void Module::UpdateFlows()
{
	float netOutwardFlow = 0;
	short numFiringThrusters = 0;

	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if ((*faceIt)->GetFiring())
		{
			(*faceIt)->SetFlowDirection(OUT);
			(*faceIt)->SetFlowRate(MAX_FLOW_RATE);

			netOutwardFlow += MAX_FLOW_RATE;
			numFiringThrusters ++;


			////Check neighbouring module
			//if (m_robot.lock()->IsModuleHere(m_position + (*faceIt)->GetNormalVector()))
			//{
			//	float neighbouringFlow = m_robot.lock()->GetModuleFlowRate(m_position + (*faceIt)->GetNormalVector());
			//	short neighbouringDirection = m_robot.lock()->GetModuleFlowDirection(m_position + (*faceIt)->GetNormalVector());
			//}

			//else
			//{	

			//	(*faceIt)->SetFlowDirection(OUT);
			//	(*faceIt)->SetFlowRate(MAX_FLOW_RATE);

			//	netOutwardFlow += MAX_FLOW_RATE;
			//	numFiringThrusters ++;
			//}
		}
	}

	float inwardFlowPerFace = netOutwardFlow / float(4 - numFiringThrusters);

	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if (!(*faceIt)->GetFiring())
		{
			(*faceIt)->SetFlowDirection(IN);
			(*faceIt)->SetFlowRate(inwardFlowPerFace);
		}
	}
}

void Module::SetZeroFlow()
{
	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		(*faceIt)->SetFlowRate(0);
	}
}

sf::Vector2f Module::GetLinearForce()
{
	//std::cout << std::endl << "Module: " << m_position.x << " " << m_position.y << std::endl;

	sf::Vector2f linearForce = sf::Vector2f(0, 0);

	//std::cout << "Num faces: " << m_faces.size() << std::endl;

	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		//std::cout << std::endl << "Iterating" << std::endl;

		sf::Vector2f forceToAdd = (*faceIt)->GetLinearForce();

		linearForce += forceToAdd;

		//std::cout << "Position: " << m_position.x << " " << m_position.y << " Lin force: " << forceToAdd.x << " " << forceToAdd.y << std::endl;

		/*if (linearForce.x != 0 || linearForce.y !=0)
		{
			char potato = 'n';

			while (potato != 'g')
			{
				std::cin >> potato;
			}
		}*/

		
	}

	//std::cout << "Force: " << linearForce.x << " " << linearForce.y << std::endl;

	//std::cout << "Finished" << std::endl;

	return linearForce;
}

float Module::GetAngularForce()
{
	float angularForce = 0;

	for (std::vector<std::shared_ptr<Face>>::iterator faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		angularForce += (*faceIt)->GetAngularForce();
	}

	return angularForce;
}

//std::shared_ptr<Face> Module::GetFace(short ID)
//{
//	return m_faces[ID];
//}

int Module::GetFaceFlowID(int face)
{
	return m_faces[face]->GetFlowID();
}

void Module::TrySetPressure(int pressureID, float pressure)
{
	if (m_pressureID == pressureID)
	{
		m_pressure = pressure;
	}
}

void Module::TrySetFlow(int flowID, float flow)
{
	if (m_faces[UP]->GetFlowID() == flowID)
	{
		if (flow > 0)
		{
			m_faces[UP]->SetFlowDirection(OUT);
		}

		else
		{
			m_faces[UP]->SetFlowDirection(IN);
		}

		//std::cout << "Setting flow up" << flow << std::endl;

		m_faces[UP]->SetFlowRate(fabs(flow));
	}

	else if (m_faces[LEFT]->GetFlowID() == flowID)
	{
		if (flow > 0)
		{
			m_faces[LEFT]->SetFlowDirection(OUT);
		}

		else
		{
			m_faces[LEFT]->SetFlowDirection(IN);
		}

		//std::cout << "Setting flow left" << flow << std::endl;

		m_faces[LEFT]->SetFlowRate(fabs(flow));
	}

	else if (m_faces[DOWN]->GetFlowID() == flowID)
	{
		if (flow > 0)
		{
			m_faces[DOWN]->SetFlowDirection(IN);
		}

		else
		{
			m_faces[DOWN]->SetFlowDirection(OUT);
		}

		//std::cout << "Setting flow down" << flow << std::endl;

		m_faces[DOWN]->SetFlowRate(fabs(flow));
	}

	else if (m_faces[RIGHT]->GetFlowID() == flowID)
	{
		if (flow > 0)
		{
			m_faces[RIGHT]->SetFlowDirection(IN);
		}

		else
		{
			m_faces[RIGHT]->SetFlowDirection(OUT);
		}

		//std::cout << "Setting flow right" << flow << std::endl;

		m_faces[RIGHT]->SetFlowRate(fabs(flow));
	}
}


void Module::SetFlowCheckedAll(bool checked)
{
	for(auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		(*faceIt)->SetFlowChecked(checked);
	}
}

void Module::SetFlowChecked(int face, bool checked)
{
	m_faces[face]->SetFlowChecked(checked);
}

bool Module::GetFlowChecked(int face)
{
	return m_faces[face]->GetFlowChecked();
}

bool Module::GetFaceBlocked(int face)
{
	//std::cout << "Getting blocked" << std::endl;


	//std::cout << m_faces.size() << std::endl;



	//std::cout << face << std::endl;



	return m_faces[face]->GetBlocked();

	//std::cout << "Returned blocked" << std::endl;
}

int Module::GetNumFacesBlocked()
{
	int blockedFaces = 0;

	for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); faceIt ++)
	{
		if ((*faceIt)->GetBlocked())
		{
			blockedFaces ++;
		}
	}

	return blockedFaces;
}

bool Module::HasNeighbour(int neighbour)
{
	if (neighbour == LEFT)
	{
		return m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(-1, 0));
	}

	else if (neighbour == RIGHT)
	{
		return m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(1, 0));
	}

	else if (neighbour == UP)
	{
		return m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(0, -1));
	}

	else if (neighbour == DOWN)
	{
		return m_robot.lock()->IsModuleHere(m_position + sf::Vector2i(0, 1));
	}
}

std::weak_ptr<Module> Module::GetNeighbour(int neighbour)
{
	std::weak_ptr<Module> module;

	if (HasNeighbour(neighbour))
	{
		if (neighbour == LEFT)
		{
			return m_robot.lock()->GetModule(m_position + sf::Vector2i(-1, 0));
		}

		else if (neighbour == RIGHT)
		{
			return m_robot.lock()->GetModule(m_position + sf::Vector2i(1, 0));
		}

		else if (neighbour == UP)
		{
			return m_robot.lock()->GetModule(m_position + sf::Vector2i(0, -1));
		}

		else if (neighbour == DOWN)
		{
			return m_robot.lock()->GetModule(m_position + sf::Vector2i(0, 1));
		}	
	}

	return module;
}