#include "Robot.hpp"
#include "Module.hpp"
#include "Face.hpp"

Robot::Robot(std::map<std::pair<int, int>, int> bodyPlan, sf::Vector2f position, int rotation, std::shared_ptr<Environment> environment)
{
	m_texture.loadFromFile("Images/Module.png");

	m_position = position;
	m_velocity = sf::Vector2f(0, 0);
	m_rotation = rotation;
	m_angularVelocity = 0;
	m_environment = environment;
	m_numExternalFiringFaces = 0;
	m_numExternalFaces = 0;
	m_needToUpdateFlows = false;

	CalculateSizeAndMidpoint(bodyPlan);

	SetUpSprite(bodyPlan);
}

void Robot::Update()
{
	//std::cout << "Velocity: " << m_velocity.x << " " << m_velocity.y << " RotationalVelocity: " << m_angularVelocity << std::endl;

	if (m_needToUpdateFlows)
	{
		UpdateFlows();

		m_needToUpdateFlows = false;
	}

	m_position.x += DT * (m_velocity.x * cos((m_rotation / (float(180)) * PI)) - m_velocity.y * sin((m_rotation / (float(180)) * PI)));
	m_position.y += DT * (m_velocity.x * sin((m_rotation / (float(180)) * PI)) + m_velocity.y * cos((m_rotation / (float(180)) * PI)));

	m_rotation += DT * m_angularVelocity;

	while (m_rotation < 0)
	{
		m_rotation += 360;
	}

	while (m_rotation >= 360)
	{
		m_rotation -= 360;
	}

	sf::Vector2f linearForce = sf::Vector2f(0, 0);

	float angularForce = 0;

	for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	{
		linearForce += moduleIt->second->GetLinearForce();

		//std::cout << linearForce.x << " " << linearForce.y << " ";

		angularForce += moduleIt->second->GetAngularForce();

		//std::cout << angularForce << " ";

		moduleIt->second->SetFlowCheckedAll(true);


		//std::cout << angularForce << " ";


	}

	//std::cout << std::endl;

	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		modIt->second->SetFlowCheckedAll(false);
	}

	//std::cout << std::endl;

	m_velocity += (DT / float(m_mass)) * linearForce;

	m_angularVelocity += (DT / m_momentOfInertia) * angularForce;

	//m_angularVelocity += (DT / float(64 * 64 * m_mass / 6)) * angularForce;

	//m_angularVelocity += (DT / float(m_mass)) * angularForce;
}

void Robot::UpdateFlows()
{
	
	//Matrix to be solved
	std::vector<std::vector<float>> matrix;

	//Fill with 0s

	for (int i = 0; i != m_numFlowEquations; i ++)
	{
		std::vector<float> row;

		for (int j = 0; j != m_numFlowEquations; j ++)
		{
			row.push_back(0);
		}

		matrix.push_back(row);
	}

	//Vector of constants
	std::vector<float> constants;

	for (int i = 0; i != m_numFlowEquations; i ++)
	{
		constants.push_back(0);
	}


	//Need to get equations and set up matrix
	//First set of equations is the Kirchoff rule equations (sum of flow rates = 0)

	//Flow assumed to flow out from module in right and down directions (positive flow)
	//flow assumed to flow into module from left and up directions (negative flow)

	//int i = 0;

	//for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	//{
	//	int upID = moduleIt->second->GetFaceFlowID(UP);
	//	int leftID = moduleIt->second->GetFaceFlowID(LEFT);
	//	int downID = moduleIt->second->GetFaceFlowID(DOWN);
	//	int rightID = moduleIt->second->GetFaceFlowID(RIGHT);

	//	//Go through each module and get the 4 flow rate IDs of the faces
	//	//So for example if the flow rate IDs are UP: 4, LEFT: 5, DOWN: 6, RIGHT: 2
	//	//The equation vector would be [0, 0, 1, 0, -1, -1, 1, 0, ... , 0]

	//	matrix[i][rightID] = 1;
	//	matrix[i][downID] = 1;
	//	matrix[i][leftID] = -1;
	//	matrix[i][upID] = -1;

	//	i ++;
	//}



	int i = 0;

	for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	{
		std::shared_ptr<Module> module = moduleIt->second;

		std::shared_ptr<Module> neighbour;

		if (module->GetNumFacesBlocked() < 3)
		{
			for (int f = 0; f != 4; f ++)
			{
				int flowID = module->GetFaceFlowID(f);

				neighbour = module->GetNeighbour(f).lock();

				if (!module->GetFaceBlocked(f))
				{
					//std::cout << "Has neighbour" << module->HasNeighbour(f) << std::endl;

					if (!module->HasNeighbour(f) || (module->HasNeighbour(f) && !neighbour->GetFaceBlocked((f + 2) % 4) && (neighbour->GetNumFacesBlocked() < 3)))
					{
						if (f == UP || f == LEFT)
						{
							matrix[i][flowID] = -1;
						}
						else
						{
							matrix[i][flowID] = 1;
						}
					}
				}
			}

			i ++;
		}
	}




	//Now are the equations equations for each face (V = IR)

	//for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	//{
	//	if (moduleIt->second->GetNumFacesBlocked() < 3)
	//	{

	//		int modulePressureID = moduleIt->second->GetPressureID();

	//		//Left face:

	//		int leftID = moduleIt->second->GetFaceFlowID(LEFT);

	//		float flowResistance;

	//		if (!moduleIt->second->GetFlowChecked(LEFT))
	//		{

	//			if (moduleIt->second->IsFaceFiring(LEFT))
	//			{
	//				constants[i] = V;

	//				flowResistance = R + S;
	//			}
	//			else 
	//			{
	//				flowResistance = R;
	//			}

	//	

	//			if (m_modules.count(std::make_pair(moduleIt->first.first - 1, moduleIt->first.second)))
	//			{
	//				//If there is a neighbouring module, then you need to add the extra flow resistance (and maybe voltage source) for the module

	//				if (m_modules[std::make_pair(moduleIt->first.first - 1, moduleIt->first.second)]->IsFaceFiring(RIGHT))
	//				{
	//					constants[i] -= V;

	//					flowResistance += R + S;
	//				}

	//				else
	//				{
	//					flowResistance += R;
	//				}



	//				matrix[i][leftID] = flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//				matrix[i][m_numFlowEquations - m_modules.size() + m_modules[std::make_pair(moduleIt->first.first - 1, moduleIt->first.second)]->GetPressureID()] = 1;

	//				m_modules[std::make_pair(moduleIt->first.first - 1, moduleIt->first.second)]->SetFlowChecked(RIGHT, true);




	//			}

	//			else
	//			{
	//				matrix[i][leftID] = flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//			}

	//			moduleIt->second->SetFlowChecked(LEFT, true);

	//		/*	std::cout << "Left face flow ID: " << leftID << " row: " << i << std::endl;

	//			for (int n = 0; n != m_numFlowEquations; n ++)
	//			{
	//				for (int m = 0; m != m_numFlowEquations; m ++)
	//				{
	//					std::cout << matrix[n][m] << " ";
	//				}

	//				std::cout << std::endl;
	//			}

	//			std::cout << std::endl;*/

	//			i ++;
	//		}



	//		//Up face:

	//		int upID = moduleIt->second->GetFaceFlowID(UP);

	//		if (!moduleIt->second->GetFlowChecked(UP))
	//		{

	//			if (moduleIt->second->IsFaceFiring(UP))
	//			{
	//				constants[i] = V;

	//				flowResistance = R + S;
	//			}
	//			else 
	//			{
	//				flowResistance = R;
	//			}

	//	
	//			if (m_modules.count(std::make_pair(moduleIt->first.first, moduleIt->first.second - 1)))
	//			{

	//				//If there is a neighbouring module, then you need to add the extra flow resistance (and maybe voltage source) for the module

	//				if (m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second - 1)]->IsFaceFiring(DOWN))
	//				{
	//					constants[i] -= V;

	//					flowResistance += R + S;
	//				}

	//				else
	//				{
	//					flowResistance += R;
	//				}


	//				matrix[i][upID] = flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//				matrix[i][m_numFlowEquations - m_modules.size() + m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second - 1)]->GetPressureID()] = 1;

	//				m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second - 1)]->SetFlowChecked(DOWN, true);
	//			}

	//			else
	//			{
	//				matrix[i][upID] = flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//			}

	//			moduleIt->second->SetFlowChecked(UP, true);

	//			/*std::cout << "Up face flow ID: " << upID << " row: " << i << std::endl;

	//			for (int n = 0; n != m_numFlowEquations; n ++)
	//			{
	//				for (int m = 0; m != m_numFlowEquations; m ++)
	//				{
	//					std::cout << matrix[n][m] << " ";
	//				}

	//				std::cout << std::endl;
	//			}

	//			std::cout << std::endl;*/

	//			i ++;

	//		}



	//		//Right face:

	//		int rightID = moduleIt->second->GetFaceFlowID(RIGHT);

	//		if (!moduleIt->second->GetFlowChecked(RIGHT))
	//		{

	//			if (moduleIt->second->IsFaceFiring(RIGHT))
	//			{
	//				constants[i] = V;

	//				flowResistance = R + S;
	//			}
	//			else 
	//			{
	//				flowResistance = R;
	//			}

	//	

	//			if (m_modules.count(std::make_pair(moduleIt->first.first + 1, moduleIt->first.second)))
	//			{

	//				//If there is a neighbouring module, then you need to add the extra flow resistance (and maybe voltage source) for the module

	//				if (m_modules[std::make_pair(moduleIt->first.first + 1, moduleIt->first.second)]->IsFaceFiring(LEFT))
	//				{
	//					constants[i] -= V;

	//					flowResistance += R + S;
	//				}

	//				else
	//				{
	//					flowResistance += R;
	//				}



	//				matrix[i][rightID] = -flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//				matrix[i][m_numFlowEquations - m_modules.size() + m_modules[std::make_pair(moduleIt->first.first + 1, moduleIt->first.second)]->GetPressureID()] = 1;

	//				m_modules[std::make_pair(moduleIt->first.first + 1, moduleIt->first.second)]->SetFlowChecked(LEFT, true);
	//			}

	//			else
	//			{
	//				matrix[i][rightID] = -flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//			}

	//			moduleIt->second->SetFlowChecked(RIGHT, true);

	//			/*std::cout << "Right face flow ID: " << rightID << " row: " << i <<  std::endl;

	//			for (int n = 0; n != m_numFlowEquations; n ++)
	//			{
	//				for (int m = 0; m != m_numFlowEquations; m ++)
	//				{
	//					std::cout << matrix[n][m] << " ";
	//				}

	//				std::cout << std::endl;
	//			}

	//			std::cout << std::endl;*/

	//			i ++;

	//		}



	//		//Down face:

	//		int downID = moduleIt->second->GetFaceFlowID(DOWN);

	//		if (!moduleIt->second->GetFlowChecked(DOWN))
	//		{

	//			if (moduleIt->second->IsFaceFiring(DOWN))
	//			{
	//				constants[i] = V;

	//				flowResistance = R + S;
	//			}
	//			else 
	//			{
	//				flowResistance = R;
	//			}

	//	

	//			if (m_modules.count(std::make_pair(moduleIt->first.first, moduleIt->first.second + 1)))
	//			{


	//				//If there is a neighbouring module, then you need to add the extra flow resistance (and maybe voltage source) for the module

	//				if (m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second + 1)]->IsFaceFiring(UP))
	//				{
	//					constants[i] -= V;

	//					flowResistance += R + S;
	//				}

	//				else
	//				{
	//					flowResistance += R;
	//				}



	//				matrix[i][downID] = -flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//				matrix[i][m_numFlowEquations - m_modules.size() + m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second + 1)]->GetPressureID()] = 1;

	//				m_modules[std::make_pair(moduleIt->first.first, moduleIt->first.second + 1)]->SetFlowChecked(UP, true);
	//			}

	//			else
	//			{
	//				matrix[i][downID] = -flowResistance;
	//				matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
	//			}

	//			moduleIt->second->SetFlowChecked(DOWN, true);

	//			/*std::cout << "Down face flow ID: " << downID << " row: " << i << std::endl;

	//			for (int n = 0; n != m_numFlowEquations; n ++)
	//			{
	//				for (int m = 0; m != m_numFlowEquations; m ++)
	//				{
	//					std::cout << matrix[n][m] << " ";
	//				}

	//				std::cout << std::endl;
	//			}

	//			std::cout << std::endl;*/

	//			i ++;

	//		}
	//	}
	//}




	for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	{
		if (moduleIt->second->GetNumFacesBlocked() < 3)
		{

			std::shared_ptr<Module> module = moduleIt->second;

			int modulePressureID = module->GetPressureID();


			//Iterate through faces
			for (int f = 0; f != 4; f ++)
			{
				std::shared_ptr<Module> neighbour;

				int flowID = module->GetFaceFlowID(f);

				neighbour = module->GetNeighbour(f).lock();

				float flowResistance;

				if (!module->GetFlowChecked(f) && !module->GetFaceBlocked(f) && (!module->HasNeighbour(f) || (module->HasNeighbour(f) && !neighbour->GetFaceBlocked((f + 2) % 4) && (neighbour->GetNumFacesBlocked() < 3))))
				{
					if (module->IsFaceFiring(f))
					{
						constants[i] = V;

						flowResistance = R + S;
					}
					else 
					{
						flowResistance = R;
					}

					if (module->HasNeighbour(f))
					{
						//If there is a neighbouring module, then you need to add the extra flow resistance (and maybe voltage source) for the module

						if (neighbour->IsFaceFiring((f + 2) % 4))
						{
							constants[i] -= V;

							flowResistance += R + S;
						}

						else
						{
							flowResistance += R;
						}


						if (f == DOWN || f == RIGHT)
						{
							matrix[i][flowID] = -flowResistance;
						}
						else
						{
							matrix[i][flowID] = flowResistance;
						}

						matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
						matrix[i][m_numFlowEquations - m_modules.size() + neighbour->GetPressureID()] = 1;

						neighbour->SetFlowChecked((f + 2) % 4, true);




					}

					else
					{
						if (f == DOWN || f == RIGHT)
						{
							matrix[i][flowID] = -flowResistance;
						}
						else
						{
							matrix[i][flowID] = flowResistance;
						}

						matrix[i][m_numFlowEquations - m_modules.size() + modulePressureID] = -1;
					}

					module->SetFlowChecked(f, true);

				/*	std::cout << "Left face flow ID: " << leftID << " row: " << i << std::endl;

					for (int n = 0; n != m_numFlowEquations; n ++)
					{
						for (int m = 0; m != m_numFlowEquations; m ++)
						{
							std::cout << matrix[n][m] << " ";
						}

						std::cout << std::endl;
					}

					std::cout << std::endl;*/

					i ++;
				}
			}
		}
	}





	//So now you have the matrix and the constants
	//Perform Gaussian elimination to find flows and pressures

	//Create Eigen matrices

	Eigen::MatrixXd eigenMatrix(m_numFlowEquations, m_numFlowEquations);

	Eigen::VectorXd eigenVector(m_numFlowEquations);

	Eigen::VectorXd solutions;

	for (int i = 0; i != m_numFlowEquations; i++)
	{
		eigenVector[i] = constants[i];
	}


	for (int i = 0; i != m_numFlowEquations; i++)
	{
		for (int j = 0; j != m_numFlowEquations; j++)
		{
			eigenMatrix(i, j) = matrix[i][j];
		}

	}

	solutions = eigenMatrix.colPivHouseholderQr().solve(eigenVector);



	std::cout << eigenMatrix << std::endl << std::endl << eigenVector << std::endl << std::endl << solutions << std::endl << std::endl;






	//Set current flows to faces

	for (int i = 0; i != m_numFlowEquations - m_modules.size(); i++)
	{
		for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
		{
			modIt->second->TrySetFlow(i, solutions[i]);
		}
	}

	//Set pressures to modules

	for (int i = m_numFlowEquations - m_modules.size(); i != m_numFlowEquations; i++)
	{
		for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
		{
			modIt->second->TrySetPressure(i + m_modules.size() - m_numFlowEquations, solutions[i]);
		}
	}






	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		modIt->second->SetFlowCheckedAll(false);
	}












	/*std::vector<std::pair<int, int>> firingModules;

	for (std::map<std::pair<int, int>, std::shared_ptr<Module>>::iterator moduleIt = m_modules.begin(); moduleIt != m_modules.end(); moduleIt ++)
	{
		moduleIt->second->SetZeroFlow();

		if(moduleIt->second->IsAnyFaceFiring())
		{
			firingModules.push_back(moduleIt->first);
		}
	}

	for (std::vector<std::pair<int, int>>::iterator moduleIt = firingModules.begin(); moduleIt != firingModules.end(); moduleIt ++)
	{
		m_modules[*moduleIt]->UpdateFlows();
	}*/
}

void Robot::Draw(sf::RenderWindow* window)
{
	m_sprite.setPosition(m_position.x, m_position.y);
	m_sprite.setRotation(m_rotation);

	window->draw(m_sprite);
}

void Robot::DrawStatic(sf::RenderWindow* window)
{
	m_sprite.setPosition(0, 0);
	m_sprite.setRotation(0);

	window->draw(m_sprite);
}

void Robot::CalculateSizeAndMidpoint(std::map<std::pair<int, int>, int> bodyPlan)
{
	int maxx, maxy;

	maxx = 0;
	maxy = 0;

	float xCoM, yCoM;

	xCoM = 0;
	yCoM = 0;

	for (std::map<std::pair<int, int>, int>::iterator it = bodyPlan.begin(); it != bodyPlan.end(); it++)
	{
		xCoM += it->first.first;
		yCoM += it->first.second;

		if (it->first.first > maxx)
		{
			maxx = it->first.first;
		}
		
		if (it->first.second > maxy)
		{
			maxy = it->first.second;
		}

		//Check to see if each module has external faces

		if (!bodyPlan.count(std::make_pair(it->first.first, it->first.second + 1)))
		{
			m_numExternalFaces ++;
		}

		if (!bodyPlan.count(std::make_pair(it->first.first, it->first.second - 1)))
		{
			m_numExternalFaces ++;
		}

		if (!bodyPlan.count(std::make_pair(it->first.first + 1, it->first.second)))
		{
			m_numExternalFaces ++;
		}

		if (!bodyPlan.count(std::make_pair(it->first.first - 1, it->first.second)))
		{
			m_numExternalFaces ++;
		}
	}

	m_mass = bodyPlan.size() * MODULE_MASS;

	xCoM = xCoM / bodyPlan.size();
	yCoM = yCoM / bodyPlan.size();

	m_realSize.x = (maxx + 1) * MODULE_SIZE;
	m_realSize.y = (maxy + 1) * MODULE_SIZE;

	m_size.x = maxx + 1;
	m_size.y = maxy + 1;

	m_realCoM.x = (xCoM + 0.5) * MODULE_SIZE;
	m_realCoM.y = (yCoM + 0.5) * MODULE_SIZE;

	m_CoM.x = xCoM + 0.5;
	m_CoM.y = yCoM + 0.5;

	std::cout << "CoM: " << m_CoM.x << " " << m_CoM.y << std::endl;

	m_realMidpoint.x = (((maxx + 1) * MODULE_SIZE) / 2);
	m_realMidpoint.y = (((maxy + 1) * MODULE_SIZE) / 2);

	m_midpoint.x = ((maxx + 1) / 2);
	m_midpoint.y = ((maxy + 1) / 2);

	m_momentOfInertia = 0;

	for (std::map<std::pair<int, int>, int>::iterator it = bodyPlan.begin(); it != bodyPlan.end(); it++)
	{
		sf::Vector2f delta = sf::Vector2f(it->first.first - m_CoM.x, it->first.second - m_CoM.y);

		m_momentOfInertia += MODULE_MASS * MODULE_SIZE * MODULE_SIZE * 0.01 * (delta.y * delta.y + delta.y + float(1)/float(3) + delta.x * delta.x + delta.x + float(1)/float(3));
	}


	if (m_realSize.x > m_realSize.y)
	{
		m_characteristicSize = 1.5 * m_realSize.x;
	}
	else
	{
		m_characteristicSize = 1.5 * m_realSize.y;
	}

}

void Robot::SetUpSprite(std::map<std::pair<int, int>, int> bodyPlan)
{
	m_renderTexture.create(m_realSize.x, m_realSize.y);

	for (std::map<std::pair<int, int>, int>::iterator modIt = bodyPlan.begin(); modIt != bodyPlan.end(); modIt ++)
	{
		sf::Sprite sprite;
		sprite.setTexture(m_texture);

		sf::IntRect rect;

		rect.height = MODULE_SIZE;
		rect.width = MODULE_SIZE;
		rect.top = 0;
		rect.left = 0;

		sprite.setTextureRect(rect);
		sprite.setPosition(MODULE_SIZE * modIt->first.first, MODULE_SIZE * modIt->first.second);

		m_renderTexture.draw(sprite);
	}

	m_sprite.setTexture(m_renderTexture.getTexture());

	m_sprite.scale(1, -1);

	m_sprite.setOrigin(m_realMidpoint.x, m_realMidpoint.y);
}

void Robot::SetSpriteFaceBlocked(sf::Vector2f modulePosition, short direction)
{
	sf::Sprite sprite;

	sprite.setTexture(m_texture);

	sf::IntRect rect;

	rect.height = 26;
	rect.width = 12;
	rect.top = 13;
	rect.left = 88;

	sprite.setTextureRect(rect);

	sprite.setOrigin(sf::Vector2f(6, 13));

	sf::Vector2f position;

	position.x = (modulePosition.x + 0.5) * MODULE_SIZE;
	position.y = (modulePosition.y + 0.5) * MODULE_SIZE;

	if (direction == UP)
	{
		position.y -= WATER_SPRITE_OFFSET;
	}

	else if (direction == RIGHT)
	{
		position.x += WATER_SPRITE_OFFSET;
		sprite.setRotation(90);
	}

	else if (direction == DOWN)
	{
		position.y += WATER_SPRITE_OFFSET;
	}

	else if (direction == LEFT)
	{
		position.x -= WATER_SPRITE_OFFSET;
		sprite.setRotation(90);
	}

	if ((direction == LEFT || direction == DOWN))
	{
		sprite.rotate(180);
	}

	sprite.setPosition(position);

	m_renderTexture.draw(sprite);

	m_sprite.setTexture(m_renderTexture.getTexture());
}



void Robot::SetSpriteFaceUnblocked(sf::Vector2f modulePosition, short direction)
{
	//Not needed
}

void Robot::SetSpriteFaceFiring(sf::Vector2f modulePosition, short direction)
{
	sf::Sprite sprite;

	sprite.setTexture(m_texture);

	sf::IntRect rect;

	rect.top = 0;
	rect.left = 65;
	rect.width = 12;
	rect.height = 12;

	sprite.setTextureRect(rect);

	sprite.setOrigin(sf::Vector2f(6, 6));

	sprite.setColor(sf::Color(0, 200, 0));

	sf::Vector2f position;

	position.x = (modulePosition.x + 0.5) * MODULE_SIZE;
	position.y = (modulePosition.y + 0.5) * MODULE_SIZE;

	if (direction == UP)
	{
		position.y -= PUMP_SPRITE_OFFSET;
		position.x += PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == RIGHT)
	{
		position.x += PUMP_SPRITE_OFFSET;
		position.y += PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == DOWN)
	{
		position.y += PUMP_SPRITE_OFFSET;
		position.x -= PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == LEFT)
	{
		position.x -= PUMP_SPRITE_OFFSET;
		position.y -= PUMP_SPRITE_OFFSET_2;
	}

	sprite.setPosition(position);

	m_renderTexture.draw(sprite);

	m_sprite.setTexture(m_renderTexture.getTexture());
}

void Robot::SetSpriteFaceNotFiring(sf::Vector2f modulePosition, short direction)
{
	sf::Sprite sprite;

	sprite.setTexture(m_texture);

	sf::IntRect rect;

	rect.top = 0;
	rect.left = 65;
	rect.width = 12;
	rect.height = 12;

	sprite.setTextureRect(rect);

	sprite.setOrigin(sf::Vector2f(6, 6));

	sprite.setColor(sf::Color(200, 0, 0));

	sf::Vector2f position;

	position.x = (modulePosition.x + 0.5) * MODULE_SIZE;
	position.y = (modulePosition.y + 0.5) * MODULE_SIZE;

	if (direction == UP)
	{
		position.y -= PUMP_SPRITE_OFFSET;
		position.x += PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == RIGHT)
	{
		position.x += PUMP_SPRITE_OFFSET;
		position.y += PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == DOWN)
	{
		position.y += PUMP_SPRITE_OFFSET;
		position.x -= PUMP_SPRITE_OFFSET_2;
	}

	else if (direction == LEFT)
	{
		position.x -= PUMP_SPRITE_OFFSET;
		position.y -= PUMP_SPRITE_OFFSET_2;
	}

	sprite.setPosition(position);

	m_renderTexture.draw(sprite);

	m_sprite.setTexture(m_renderTexture.getTexture());
}

void Robot::SetSpriteFlowRate(sf::Vector2f modulePosition, short direction, short flowDirection, float flowStrength)
{
	sf::Sprite sprite;

	sprite.setTexture(m_texture);

	sf::IntRect rect;

	rect.height = 26;
	rect.width = 12;
	rect.top = 13;

	if (flowStrength > 0.0000000000001)
	{
		rect.left = 65;
	}

	else
	{
		rect.left = 76;
	}

	sprite.setTextureRect(rect);

	sprite.setOrigin(sf::Vector2f(6, 13));

	//if (flowStrength > 0)
	//{
		sprite.setColor(sf::Color(-flowStrength * 60 + RED, GREEN, BLUE));
	//}

	sf::Vector2f position;

	position.x = (modulePosition.x + 0.5) * MODULE_SIZE;
	position.y = (modulePosition.y + 0.5) * MODULE_SIZE;

	if (direction == UP)
	{
		position.y -= WATER_SPRITE_OFFSET;
	}

	else if (direction == RIGHT)
	{
		position.x += WATER_SPRITE_OFFSET;
		sprite.setRotation(90);
	}

	else if (direction == DOWN)
	{
		position.y += WATER_SPRITE_OFFSET;
	}

	else if (direction == LEFT)
	{
		position.x -= WATER_SPRITE_OFFSET;
		sprite.setRotation(90);
	}

	if (flowDirection == OUT && (direction == LEFT || direction == DOWN))
	{
		sprite.rotate(180);
	}

	if (flowDirection == IN && (direction == RIGHT || direction == UP))
	{
		sprite.rotate(180);
	}

	sprite.setPosition(position);

	m_renderTexture.draw(sprite);

	m_sprite.setTexture(m_renderTexture.getTexture());
}

void Robot::SetUpModules(std::map<std::pair<int, int>, int> bodyPlan)
{
	int IDcounter = 0;

	for (std::map<std::pair<int, int>, int>::iterator modIt = bodyPlan.begin(); modIt != bodyPlan.end(); modIt ++)
	{
		m_modules[std::make_pair(modIt->first.first, modIt->first.second)] = std::shared_ptr<Module>(new Module(sf::Vector2i(modIt->first.first, modIt->first.second), IDcounter, shared_from_this(), m_texture));

		IDcounter ++;
	}

	////This contains the set of modules that have already been set up (and the faces assigned flow IDs)
	//std::set<std::pair<int, int>> alreadySet;

	//int flowIDcounter = 0;

	//for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	//
	////for (std::map<std::pair<int, int>, int>::iterator modIt = bodyPlan.begin(); modIt != bodyPlan.end(); modIt ++)
	//{
	//	//Get face flow IDs, going UP, RIGHT, DOWN, LEFT
	//	//Flow assumed to flow out from module in right and down directions (positive flow)
	//	//flow assumed to flow into module from left and up directions (negative flow)

	//	std::shared_ptr<Module> module = modIt->second;

	//	int upID, rightID, downID, leftID;

	//	//UP
	//	if (!module->HasNeighbour(UP))
	//	//if (!m_modules.count(std::make_pair(modIt->first.first, modIt->first.second - 1)))
	//	{
	//		//If there is no module there, then it is definitely a new flow
	//		upID = flowIDcounter;

	//		flowIDcounter ++;
	//	}

	//	else
	//	{
	//		//There is a module there, check if has already been set up
	//		if (alreadySet.count(std::make_pair(modIt->first.first, modIt->first.second - 1)))
	//		{
	//			//If it has, get the flow ID for the corresponding face
	//			upID = m_modules[std::make_pair(modIt->first.first, modIt->first.second - 1)]->GetFace(DOWN).lock()->GetFlowID();
	//		}

	//		else
	//		{
	//			//If it hasn't, new flow ID
	//			upID = flowIDcounter;

	//			flowIDcounter ++;
	//		}
	//	}




	//	//RIGHT

	//	if (!m_modules.count(std::make_pair(modIt->first.first + 1, modIt->first.second)))
	//	{
	//		//If there is no module there, then it is definitely a new flow
	//		rightID = flowIDcounter;

	//		flowIDcounter ++;
	//	}

	//	else
	//	{
	//		//There is a module there, check if has already been set up
	//		if (alreadySet.count(std::make_pair(modIt->first.first + 1, modIt->first.second)))
	//		{
	//			//If it has, get the flow ID for the corresponding face
	//			rightID = m_modules[std::make_pair(modIt->first.first + 1, modIt->first.second)]->GetFace(LEFT).lock()->GetFlowID();
	//		}

	//		else
	//		{
	//			//If it hasn't, new flow ID
	//			rightID = flowIDcounter;

	//			flowIDcounter ++;
	//		}
	//	}




	//	//DOWN

	//	if (!m_modules.count(std::make_pair(modIt->first.first, modIt->first.second + 1)))
	//	{
	//		//If there is no module there, then it is definitely a new flow
	//		downID = flowIDcounter;

	//		flowIDcounter ++;
	//	}

	//	else
	//	{
	//		//There is a module there, check if has already been set up
	//		if (alreadySet.count(std::make_pair(modIt->first.first, modIt->first.second + 1)))
	//		{
	//			//If it has, get the flow ID for the corresponding face
	//			downID = m_modules[std::make_pair(modIt->first.first, modIt->first.second + 1)]->GetFace(UP).lock()->GetFlowID();
	//		}

	//		else
	//		{
	//			//If it hasn't, new flow ID
	//			downID = flowIDcounter;

	//			flowIDcounter ++;
	//		}
	//	}






	//	//LEFT

	//	if (!m_modules.count(std::make_pair(modIt->first.first - 1, modIt->first.second)))
	//	{
	//		//If there is no module there, then it is definitely a new flow
	//		leftID = flowIDcounter;

	//		flowIDcounter ++;
	//	}

	//	else
	//	{
	//		//There is a module there, check if has already been set up
	//		if (alreadySet.count(std::make_pair(modIt->first.first - 1, modIt->first.second)))
	//		{
	//			//If it has, get the flow ID for the corresponding face
	//			leftID = m_modules[std::make_pair(modIt->first.first - 1, modIt->first.second)]->GetFace(RIGHT).lock()->GetFlowID();
	//		}

	//		else
	//		{
	//			//If it hasn't, new flow ID
	//			leftID = flowIDcounter;

	//			flowIDcounter ++;
	//		}
	//	}








	//	m_modules[std::make_pair(modIt->first.first, modIt->first.second)]->SetUpFaces(upID, rightID, downID, leftID);

	//	alreadySet.insert(modIt->first);
	//}

	//Calculate total number of equations for flow calculations
	//One equation per module + 1 equation per 'connection' (including those to the environment)

	CalculateFlowIDs();

	CalculateNumFlowEquations();
}

void Robot::SelectModule(sf::Vector2f position, int command)
{
	sf::Vector2i moduleCoords;

	moduleCoords.x = (m_realCoM.x + position.x) / MODULE_SIZE;
	moduleCoords.y = (m_realCoM.y + position.y) / MODULE_SIZE;

	if (m_modules.count(std::make_pair(moduleCoords.x, moduleCoords.y)))
	{
		sf::Vector2f moduleRelativePosition;

		moduleRelativePosition.x = int(m_realCoM.x + position.x) % MODULE_SIZE - (MODULE_SIZE / 2);
		moduleRelativePosition.y = int(m_realCoM.y + position.y) % MODULE_SIZE - (MODULE_SIZE / 2);

		m_modules[std::make_pair(moduleCoords.x, moduleCoords.y)]->SelectFace(moduleRelativePosition, command);
	}
}

bool Robot::CanFireExternalFace()
{
	if (m_numExternalFiringFaces == m_numExternalFaces / 2)
	{
		return false;
	}

	return true;
}

void Robot::FireExternalFace()
{
	m_numExternalFiringFaces ++;
}

void Robot::StopFiringExternalFace()
{
	m_numExternalFiringFaces --;
}

bool Robot::IsModuleHere(sf::Vector2i position)
{
	return m_modules.count(std::make_pair(position.x, position.y));
}

float Robot::GetModuleFlowRate(sf::Vector2i position)
{
	return 0;
	//return m_modules[std::make_pair(position.x, position.y)]->Get
}

short Robot::GetModuleFlowDirection(sf::Vector2i position)
{
	return 0;
}


void Robot::CalculateFlowIDs()
{
	//This contains the set of modules that have already been set up (and the faces assigned flow IDs)
	std::set<std::pair<int, int>> alreadySet;

	int flowIDcounter = 0;

	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		//Get face flow IDs, going UP, RIGHT, DOWN, LEFT
		//Flow assumed to flow out from module in right and down directions (positive flow)
		//flow assumed to flow into module from left and up directions (negative flow)

		std::shared_ptr<Module> module = modIt->second;

		int upID, rightID, downID, leftID;

		//UP

		if (!module->HasNeighbour(UP))
		{
			//If there is no module there, then it is definitely a new flow
			upID = flowIDcounter;

			flowIDcounter ++;
		}

		else
		{
			//There is a module there, check if has already been set up
			if (alreadySet.count(std::make_pair(modIt->first.first, modIt->first.second - 1)))
			{
				//If it has, get the flow ID for the corresponding face
				upID = module->GetNeighbour(UP).lock()->GetFace(DOWN).lock()->GetFlowID();
			}

			else
			{
				//If it hasn't, new flow ID
				upID = flowIDcounter;

				flowIDcounter ++;
			}
		}




		//RIGHT

		if (!module->HasNeighbour(RIGHT))
		{
			//If there is no module there, then it is definitely a new flow
			rightID = flowIDcounter;

			flowIDcounter ++;
		}

		else
		{
			//There is a module there, check if has already been set up
			if (alreadySet.count(std::make_pair(modIt->first.first + 1, modIt->first.second)))
			{
				//If it has, get the flow ID for the corresponding face
				rightID = module->GetNeighbour(RIGHT).lock()->GetFace(LEFT).lock()->GetFlowID();
			}

			else
			{
				//If it hasn't, new flow ID
				rightID = flowIDcounter;

				flowIDcounter ++;
			}
		}




		//DOWN

		if (!module->HasNeighbour(DOWN))
		{
			//If there is no module there, then it is definitely a new flow
			downID = flowIDcounter;

			flowIDcounter ++;
		}

		else
		{
			//There is a module there, check if has already been set up
			if (alreadySet.count(std::make_pair(modIt->first.first, modIt->first.second + 1)))
			{
				//If it has, get the flow ID for the corresponding face
				downID = module->GetNeighbour(DOWN).lock()->GetFace(UP).lock()->GetFlowID();
			}

			else
			{
				//If it hasn't, new flow ID
				downID = flowIDcounter;

				flowIDcounter ++;
			}
		}






		//LEFT

		if (!module->HasNeighbour(LEFT))
		{
			//If there is no module there, then it is definitely a new flow
			leftID = flowIDcounter;

			flowIDcounter ++;
		}

		else
		{
			//There is a module there, check if has already been set up
			if (alreadySet.count(std::make_pair(modIt->first.first - 1, modIt->first.second)))
			{
				//If it has, get the flow ID for the corresponding face
				leftID = module->GetNeighbour(LEFT).lock()->GetFace(RIGHT).lock()->GetFlowID();
			}

			else
			{
				//If it hasn't, new flow ID
				leftID = flowIDcounter;

				flowIDcounter ++;
			}
		}

		m_modules[std::make_pair(modIt->first.first, modIt->first.second)]->SetUpFaces(upID, rightID, downID, leftID);

		alreadySet.insert(modIt->first);
	}
}

void Robot::CalculateNumFlowEquations()
{
	m_numFlowEquations = 0;

	for (auto it = m_modules.begin(); it != m_modules.end(); it++)
	{
		int numFacesUnblocked = 0;

		int numPressureEquationsAdded = 0;

		//LEFT

		if (!it->second->GetFaceBlocked(LEFT))
		{
			//Face is unblocked, mark as checked

			it->second->SetFlowChecked(LEFT, true);

			numFacesUnblocked ++;

			//Check if the face has a neighbour
			if (m_modules.count(std::make_pair(it->first.first - 1, it->first.second)))
			{
				//If is has, check if neighbour has already been checked or if neighour is blocked (or only has 1 open face)
				if(!m_modules[std::make_pair(it->first.first - 1, it->first.second)]->GetFlowChecked(RIGHT) && !m_modules[std::make_pair(it->first.first - 1, it->first.second)]->GetFaceBlocked(RIGHT) && (m_modules[std::make_pair(it->first.first - 1, it->first.second)]->GetNumFacesBlocked() < 3))
				{
					//If not, add equation

					numPressureEquationsAdded ++;
				}
			}

			else
			{
				//If not, add equation

				numPressureEquationsAdded ++;
			}

		}

		//RIGHT

		if (!it->second->GetFaceBlocked(RIGHT))
		{
			//Face is unblocked, mark as checked

			it->second->SetFlowChecked(RIGHT, true);

			numFacesUnblocked ++;

			//Check if the face has a neighbour
			if (m_modules.count(std::make_pair(it->first.first + 1, it->first.second)))
			{
				//If is has, check if neighbour has already been checked or if neighour is blocked
				if(!m_modules[std::make_pair(it->first.first + 1, it->first.second)]->GetFlowChecked(LEFT) && !m_modules[std::make_pair(it->first.first + 1, it->first.second)]->GetFaceBlocked(LEFT) && (m_modules[std::make_pair(it->first.first + 1, it->first.second)]->GetNumFacesBlocked() < 3))
				{
					//If not, add equation

					numPressureEquationsAdded ++;
				}
			}

			else
			{
				//If not, add equation

				numPressureEquationsAdded ++;
			}

		}



		//UP

		if (!it->second->GetFaceBlocked(UP))
		{
			//Face is unblocked, mark as checked

			it->second->SetFlowChecked(UP, true);

			numFacesUnblocked ++;

			//Check if the face has a neighbour
			if (m_modules.count(std::make_pair(it->first.first, it->first.second - 1)))
			{
				//If is has, check if neighbour has already been checked or if neighour is blocked
				if(!m_modules[std::make_pair(it->first.first, it->first.second - 1)]->GetFlowChecked(DOWN) && !m_modules[std::make_pair(it->first.first, it->first.second - 1)]->GetFaceBlocked(DOWN) && (m_modules[std::make_pair(it->first.first, it->first.second - 1)]->GetNumFacesBlocked() < 3))
				{
					//If not, add equation

					numPressureEquationsAdded ++;
				}
			}

			else
			{
				//If not, add equation

				numPressureEquationsAdded ++;
			}

		}




		//DOWN

		if (!it->second->GetFaceBlocked(DOWN))
		{
			//Face is unblocked, mark as checked

			it->second->SetFlowChecked(DOWN, true);

			numFacesUnblocked ++;

			//Check if the face has a neighbour
			if (m_modules.count(std::make_pair(it->first.first, it->first.second + 1)))
			{
				//If is has, check if neighbour has already been checked or if neighour is blocked
				if(!m_modules[std::make_pair(it->first.first, it->first.second + 1)]->GetFlowChecked(UP) && !m_modules[std::make_pair(it->first.first, it->first.second + 1)]->GetFaceBlocked(UP) && (m_modules[std::make_pair(it->first.first, it->first.second + 1)]->GetNumFacesBlocked() < 3))
				{
					//If not, add equation

					numPressureEquationsAdded ++;
				}
			}

			else
			{
				//If not, add equation

				numPressureEquationsAdded ++;
			}

		}











		//If the module isn't a dead end, add an equation for krichoff rule
		if (numFacesUnblocked > 1)
		{
			m_numFlowEquations += 1 + numPressureEquationsAdded;
		}


	}

	for (auto modIt = m_modules.begin(); modIt != m_modules.end(); modIt ++)
	{
		modIt->second->SetFlowCheckedAll(false);
	}

	std::cout << "NumFlowEquations: " << m_numFlowEquations << std::endl;

}