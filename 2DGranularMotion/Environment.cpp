#include "Environment.hpp"
#include "Robot.hpp"

void Environment::Update()
{
	if (!m_paused)
	{
		for (std::vector<std::shared_ptr<Robot>>::iterator robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
		{
			(*robotIt)->Update();
		}
	}
}

void Environment::Draw(sf::RenderWindow* window)
{
	window->clear(sf::Color(RED, GREEN, BLUE));

	if (!m_viewLocked)
	{
		for (std::vector<std::shared_ptr<Robot>>::iterator robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
		{
			(*robotIt)->Draw(window);
		}
	}
	else
	{
		for (std::vector<std::shared_ptr<Robot>>::iterator robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
		{
			(*robotIt)->DrawStatic(window);
		}
	}

	for (int i = 0; i != m_arrowDirections.size(); i ++)
	{
		DrawArrow(window, m_arrowDirections[i], m_arrowOrigins[i], m_arrowColors[i]);
	}

	m_arrowDirections.clear();
	m_arrowOrigins.clear();
	m_arrowColors.clear();

	sf::View mainView = window->getView();

	window->setView(window->getDefaultView());

	window->setView(mainView);

	window->display();
}

void Environment::HandleEvents(sf::RenderWindow* window)
{
	sf::View view = window->getView();

	float zoomRatio = SCREEN_WIDTH / view.getSize().x;

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
	{
		view.move(sf::Vector2f(-SCROLL_FACTOR / (float(FPS)) / zoomRatio, 0));
	}

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
	{
		view.move(sf::Vector2f(SCROLL_FACTOR / (float(FPS)) / zoomRatio, 0));
	}

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
	{
		view.move(sf::Vector2f(0, -SCROLL_FACTOR / (float(FPS)) / zoomRatio));
	}

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
	{
		view.move(sf::Vector2f(0, SCROLL_FACTOR / (float(FPS)) / zoomRatio));
	}

	window->setView(view);

	sf::Event event;

	while(window->pollEvent(event))
	{
		switch(event.type)
		{
			case sf::Event::KeyPressed:

				switch(event.key.code)
				{
					case sf::Keyboard::V:

						std::cout << "LOCK VIEW" << std::endl;

						m_viewLocked = !m_viewLocked;
						break;

					case sf::Keyboard::P:

						std::cout << "PAUSED" << std::endl;

						m_paused = !m_paused;
						break;

					default:
						break;
				}

				break;

			case sf::Event::MouseWheelMoved:

				if (event.mouseWheel.delta > 0)
				{
					sf::Vector2f viewSize = view.getSize();

					sf::Vector2i newSize = sf::Vector2i(int(2 * viewSize.x), int(2 * viewSize.y));

					view.setSize(sf::Vector2f(newSize));
				}
				else
				{
					sf::Vector2f viewSize = view.getSize();

					sf::Vector2i newSize = sf::Vector2i(int(0.5 * viewSize.x), int(0.5 * viewSize.y));

					view.setSize(sf::Vector2f(newSize));
				}

				window->setView(view);

				break;

			case sf::Event::MouseButtonPressed:

				if (event.mouseButton.button == sf::Mouse::Left)
				{
				

					sf::Vector2f mousePositionInWorld = window->mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));

					std::cout << "MOUSE: " << mousePositionInWorld.x << " " << mousePositionInWorld.y << std::endl;

					for (std::vector<std::shared_ptr<Robot>>::iterator robotIt = m_robots.begin(); robotIt != m_robots.end(); robotIt ++)
					{
						sf::Vector2f vectorBetweenMouseAndRobot;

						if (m_viewLocked)
						{
							vectorBetweenMouseAndRobot = mousePositionInWorld;
						}
						else
						{
							vectorBetweenMouseAndRobot = mousePositionInWorld - (*robotIt)->GetPosition();
						}

						float distanceFromMouseToRobot = sqrt(pow(vectorBetweenMouseAndRobot.x, 2) + pow(vectorBetweenMouseAndRobot.y, 2));

						if((*robotIt)->GetCharacteristicSize() > distanceFromMouseToRobot)
						{
							//Rotate mouse position and pass it to robot

							sf::Vector2f transformedMousePosition;

							if (m_viewLocked)
							{
								transformedMousePosition.x = vectorBetweenMouseAndRobot.x;
								transformedMousePosition.y = vectorBetweenMouseAndRobot.y;
							}
							else
							{
								transformedMousePosition.x = vectorBetweenMouseAndRobot.x * cos(-PI * float((*robotIt)->GetRotation()) / float(180)) - vectorBetweenMouseAndRobot.y * sin(-PI * float((*robotIt)->GetRotation()) / float(180));
								transformedMousePosition.y = vectorBetweenMouseAndRobot.x * sin(-PI * float((*robotIt)->GetRotation()) / float(180)) + vectorBetweenMouseAndRobot.y * cos(-PI * float((*robotIt)->GetRotation()) / float(180));
							}

							if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
							{
								(*robotIt)->SelectModule(transformedMousePosition, TOGGLE_BLOCK);
							}
							else
							{
								(*robotIt)->SelectModule(transformedMousePosition, TOGGLE_ACTIVATE);
							}

							break;
						}
					}
				}

				break;

			default:
				break;
		}
	}
}

void Environment::AddRobot(short type, int width, int height)
{
	//The body plan that will be passed to the robot contructor
	std::map<std::pair<int, int>, int> bodyPlan;

	if (type == RECTANGULAR)
	{
		/*for (int i = 0; i != width; i ++)
		{
			for (int j = 0; j != height; j++)
			{
				bodyPlan[std::make_pair(i, j)] = 1;
			}
		}*/

		bodyPlan[std::make_pair(0, 0)] = 1;
		bodyPlan[std::make_pair(1, 0)] = 1;
		/*bodyPlan[std::make_pair(1, 1)] = 1;
		bodyPlan[std::make_pair(2, 1)] = 1;
		bodyPlan[std::make_pair(2, 0)] = 1;*/
	}

	else if (type == RANDOM)
	{
	}

	m_robots.push_back(std::shared_ptr<Robot>(new Robot(bodyPlan, sf::Vector2f(200, 200), 0, shared_from_this())));

	m_robots.back()->SetUpModules(bodyPlan);
}

void Environment::AddArrow(sf::Vector2f direction, sf::Vector2f origin, sf::Color color)
{
	m_arrowDirections.push_back(direction);
	m_arrowOrigins.push_back(origin);
	m_arrowColors.push_back(color);
}

void Environment::DrawArrow(sf::RenderWindow* window, sf::Vector2f direction, sf::Vector2f origin, sf::Color color)
{
	sf::CircleShape circleOne, circleTwo;

	sf::RectangleShape rect;

	rect.setSize(sf::Vector2f(20, 4));

	rect.setOrigin(sf::Vector2f(0, 2));



	circleOne.setFillColor(sf::Color::Black);
	circleTwo.setFillColor(color);

	circleOne.setRadius(5);
	circleTwo.setRadius(5);

	circleOne.setOrigin(sf::Vector2f(5, 5));
	circleTwo.setOrigin(sf::Vector2f(5, 5));

	direction.x *= 5;
	direction.y *= 5;

	circleOne.setPosition(origin);
	circleTwo.setPosition(origin + direction);

	window->draw(circleOne);
	window->draw(circleTwo);
}