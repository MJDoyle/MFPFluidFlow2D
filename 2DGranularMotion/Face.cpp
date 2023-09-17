#include "Face.hpp"

#include "Module.hpp"

#include "Robot.hpp"

#include "Environment.hpp"

Face::Face(int direction, int flowID, bool external, std::shared_ptr<Module> module)
{
	m_flowID = flowID;
	m_external = external;
	m_direction = direction;
	m_module = module;
	m_firing = false;
	m_flowDirection = OUT;
	m_flowRate = 0;
	m_flowChecked = false;
	m_blocked = false;

	if (direction == UP)
	{
		m_normalVector = sf::Vector2i(0, -1);
		m_CoMRelativePosition = module->GetCoMRelativePosition() + sf::Vector2f(0, -0.5);
	}

	else if (direction == RIGHT)
	{
		m_normalVector = sf::Vector2i(1, 0);
		m_CoMRelativePosition = module->GetCoMRelativePosition() + sf::Vector2f(0.5, 0);
	}

	else if (direction == DOWN)
	{
		m_normalVector = sf::Vector2i(0, 1);
		m_CoMRelativePosition = module->GetCoMRelativePosition() + sf::Vector2f(0, 0.5);
	}

	else if (direction == LEFT)
	{
		m_normalVector = sf::Vector2i(-1, 0);
		m_CoMRelativePosition = module->GetCoMRelativePosition() + sf::Vector2f(-0.5, 0);
	}

	std::cout << "Face: " << m_CoMRelativePosition.x << " " << m_CoMRelativePosition.y;

	if (m_external)
	{
		std::cout << " External";
	}

	std::cout << std::endl;

	SetFlowRate(m_flowRate);

}

void Face::SetBlocked()
{
	m_module.lock()->GetRobot().lock()->SetSpriteFaceBlocked(sf::Vector2f(m_module.lock()->GetPosition()), m_direction);
	m_blocked = true;
	m_module.lock()->GetRobot().lock()->CalculateFlowIDs();
	m_module.lock()->GetRobot().lock()->CalculateNumFlowEquations();
}

void Face::SetUnblocked()
{
	m_module.lock()->GetRobot().lock()->SetSpriteFaceUnblocked(sf::Vector2f(m_module.lock()->GetPosition()), m_direction);
	m_blocked = false;
	m_module.lock()->GetRobot().lock()->CalculateFlowIDs();
	m_module.lock()->GetRobot().lock()->CalculateNumFlowEquations();
}

void Face::SetFiring()
{
	m_module.lock()->GetRobot().lock()->SetSpriteFaceFiring(sf::Vector2f(m_module.lock()->GetPosition()), m_direction);
	m_firing = true;
}

void Face::SetNotFiring()
{
	m_module.lock()->GetRobot().lock()->SetSpriteFaceNotFiring(sf::Vector2f(m_module.lock()->GetPosition()), m_direction);
	m_firing = false;
}


void Face::SetFlowRate(float rate)
{
	m_flowRate = rate;
	if (!m_blocked)
	{
		m_module.lock()->GetRobot().lock()->SetSpriteFlowRate(sf::Vector2f(m_module.lock()->GetPosition()), m_direction, m_flowDirection, rate);
	}
}

sf::Vector2f Face::GetLinearForce()
{

	//USING JUST PUMPS

	/*if (!m_firing)
	{
		return CalculateDragForce();
	}

	else
	{
		return -PUMP_THRUST * sf::Vector2f(m_normalVector) + CalculateDragForce();
	}*/


	//USING FLOW
	return CalculateDragForce() + CalculateMovementForce();

	//return CalculateMovementForce();
}

sf::Vector2f Face::CalculateMovementForce()
{
	sf::Vector2f force = sf::Vector2f(0, 0);


	//USING CURRENT

	if ((!m_module.lock()->HasNeighbour(m_direction)) || !m_module.lock()->GetNeighbour(m_direction).lock()->GetFlowChecked((m_direction + 2) % 4))
	{

		if (m_flowDirection == IN)
		{
			force.x = m_normalVector.x * m_flowRate * THRUST_COEFFICIENT;
			force.y = m_normalVector.y * m_flowRate * THRUST_COEFFICIENT;
		}

		else
		{
			force.x = -m_normalVector.x * m_flowRate * THRUST_COEFFICIENT;
			force.y = -m_normalVector.y * m_flowRate * THRUST_COEFFICIENT;
		}
	}

	



	//USING VOLTAGE

	/*if (m_module.lock()->HasNeighbour(m_direction))
	{
		force.x = force.x * -THRUST_COEFFICIENT * (m_module.lock()->GetPressure() - m_module.lock()->GetNeighbour(m_direction).lock()->GetPressure());
		force.y = force.y * -THRUST_COEFFICIENT * (m_module.lock()->GetPressure() - m_module.lock()->GetNeighbour(m_direction).lock()->GetPressure());
	}
	else
	{
		force.x = force.x * -THRUST_COEFFICIENT * m_module.lock()->GetPressure();
		force.y = force.y * -THRUST_COEFFICIENT * m_module.lock()->GetPressure();
	}*/


	return force;
}

float Face::GetAngularForce()
{

	//USING JUST PUMPS

	//if (!m_firing)
	//{
	//	sf::Vector2f dragForce = CalculateDragForce();

	//	//return 0;
	//	return m_CoMRelativePosition.x * MODULE_SIZE * dragForce.y - m_CoMRelativePosition.y * MODULE_SIZE * dragForce.x;
	//}

	//else
	//{
	//	sf::Vector2f dragForce = CalculateDragForce();

	//	return -m_CoMRelativePosition.x * MODULE_SIZE * m_normalVector.y * PUMP_THRUST			+			m_normalVector.x * m_CoMRelativePosition.y * MODULE_SIZE * PUMP_THRUST			+			m_CoMRelativePosition.x * MODULE_SIZE * dragForce.y			-			m_CoMRelativePosition.y * MODULE_SIZE * dragForce.x;	
	//	//return -m_CoMRelativePosition.x * MODULE_SIZE * m_normalVector.y * PUMP_THRUST + m_normalVector.x * m_CoMRelativePosition.y * MODULE_SIZE * PUMP_THRUST;	
	//}


	//USING FLOW

	sf::Vector2f force = -CalculateMovementForce() - CalculateDragForce();

	//sf::Vector2f force = -CalculateMovementForce();

	return -m_CoMRelativePosition.x * MODULE_SIZE * force.y + m_CoMRelativePosition.y * MODULE_SIZE * force.x;

}

sf::Vector2f Face::CalculateDragForce()
{

	if(m_external)
	{
		sf::Vector2f velocity = m_module.lock()->GetRobot().lock()->GetVelocity();
		float rotation = m_module.lock()->GetRobot().lock()->GetRotation() * PI / float(180);
		float angularVelocity = m_module.lock()->GetRobot().lock()->GetAngularVelocity() * PI / float(180);

		sf::Vector2f velocityInRealWorld;

		velocityInRealWorld.x = cos(rotation) * velocity.x - sin(rotation) * velocity.y;
		velocityInRealWorld.y = sin(rotation) * velocity.x + cos(rotation) * velocity.y;

		//std::cout << std::endl << "Velocity in real World: " << velocityInRealWorld.x << " " << velocityInRealWorld.y << std::endl;

		sf::Vector2f totalFaceVelocityInRealWorld;

		sf::Vector2f realCoMRelativePosition;

		realCoMRelativePosition.x = m_CoMRelativePosition.x * MODULE_SIZE;
		realCoMRelativePosition.y = m_CoMRelativePosition.y * MODULE_SIZE;

		//std::cout << "Position: " << realCoMRelativePosition.x << " " << realCoMRelativePosition.y << std::endl;

		totalFaceVelocityInRealWorld.x = velocityInRealWorld.x - angularVelocity * (sin(rotation) * realCoMRelativePosition.x + cos(rotation) * realCoMRelativePosition.y);
		totalFaceVelocityInRealWorld.y = (velocityInRealWorld.y + angularVelocity * (cos(rotation) * realCoMRelativePosition.x - sin(rotation) * realCoMRelativePosition.y));

		//std::cout << "Total velocity in real World: " << totalFaceVelocityInRealWorld.x << " " << totalFaceVelocityInRealWorld.y << std::endl;

		sf::Vector2f totalFaceVelocity;

		totalFaceVelocity.x = (cos(rotation) * totalFaceVelocityInRealWorld.x + sin(rotation) * totalFaceVelocityInRealWorld.y);
		totalFaceVelocity.y = (-sin(rotation) * totalFaceVelocityInRealWorld.x + cos(rotation) * totalFaceVelocityInRealWorld.y);


		/*if (m_CoMRelativePosition.x == -2 && m_CoMRelativePosition.y == -1.5)
		{

			sf::Vector2f origin = m_module.lock()->GetRobot().lock()->GetPosition();

			float rotation1 = m_module.lock()->GetRobot().lock()->GetRotation() * PI / float(180);

			origin.x += m_CoMRelativePosition.x * MODULE_SIZE * cos(rotation1) - m_CoMRelativePosition.y * MODULE_SIZE * sin(rotation1);
			origin.y += m_CoMRelativePosition.x * MODULE_SIZE * sin(rotation1) + m_CoMRelativePosition.y * MODULE_SIZE * cos(rotation1);

			sf::Vector2f rotatedForce;

			rotatedForce.x = totalFaceVelocity.x * cos(rotation1) - totalFaceVelocity.y * sin(rotation1);
			rotatedForce.y = totalFaceVelocity.x * sin(rotation1) + totalFaceVelocity.y * cos(rotation1);

			m_module.lock()->GetRobot().lock()->GetEnvironment().lock()->AddArrow(totalFaceVelocityInRealWorld, origin, sf::Color::Blue);

			rotatedForce.x = m_normalVector.x * cos(rotation1) - m_normalVector.y * sin(rotation1);
			rotatedForce.y = m_normalVector.x * sin(rotation1) + m_normalVector.y * cos(rotation1);

			m_module.lock()->GetRobot().lock()->GetEnvironment().lock()->AddArrow(rotatedForce, origin, sf::Color::Green);

		}*/




		float cosTheta = (totalFaceVelocity.x * m_normalVector.x + totalFaceVelocity.y * m_normalVector.y) / (sqrt( (pow(totalFaceVelocity.x, 2) + (pow(totalFaceVelocity.y, 2))) * (pow(float(m_normalVector.x), 2) + (pow(float(m_normalVector.y), 2))) ));

		if (acos(cosTheta) < PI / 2)
		{
			float forceStrength = -1 * DRAG_COEFFICIENT * (pow(totalFaceVelocity.x, 2) + (pow(totalFaceVelocity.y, 2))) * cosTheta * cosTheta;

			sf::Vector2f force = sf::Vector2f(forceStrength * m_normalVector.x,  forceStrength * m_normalVector.y);	

			/*if (m_CoMRelativePosition.x == -2 && m_CoMRelativePosition.y == -1.5)
			{

				sf::Vector2f origin = m_module.lock()->GetRobot().lock()->GetPosition();

				float rotation = m_module.lock()->GetRobot().lock()->GetRotation() * PI / float(180);

				origin.x += m_CoMRelativePosition.x * MODULE_SIZE * cos(rotation) - m_CoMRelativePosition.y * MODULE_SIZE * sin(rotation);
				origin.y += m_CoMRelativePosition.x * MODULE_SIZE * sin(rotation) + m_CoMRelativePosition.y * MODULE_SIZE * cos(rotation);

				sf::Vector2f rotatedForce;

				rotatedForce.x = force.x * cos(rotation) - force.y * sin(rotation);
				rotatedForce.y = force.x * sin(rotation) + force.y * cos(rotation);

				m_module.lock()->GetRobot().lock()->GetEnvironment().lock()->AddArrow(rotatedForce, origin, sf::Color::Red);

			}*/

			return force;
		}

		return sf::Vector2f(0, 0);

	}

	return sf::Vector2f(0, 0);
}


float Face::CalculateRotationalDragForce()
{
	sf::Vector2f dragForce = CalculateDragForce();

	if (dragForce.x != 0 && dragForce.y == 0)
	{
		return dragForce.x;
	}

	else if (dragForce.x == 0 && dragForce.y != 0)
	{
		return dragForce.y;
	}

	else if (dragForce.x == 0 && dragForce.y == 0)
	{
		return 0;
	}

	else
	{
		std::cout << "Broken" << std::endl;
		
		while (1)
		{
		}
	}
}