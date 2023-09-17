#ifndef ENVIRONMENT_PP
#define ENVIRONMENT_HPP

#include "Config.hpp"

class Robot;

class Environment : public std::enable_shared_from_this<Environment>
{

public:

	Environment()
	{
		m_paused = false;
		m_viewLocked = false;
	}

	void AddRobot(short type, int width = 0, int height = 0);

	void Update();

	void Draw(sf::RenderWindow* window);

	void HandleEvents(sf::RenderWindow* window);

	void AddArrow(sf::Vector2f direction, sf::Vector2f origin, sf::Color color);

private:

	void DrawArrow(sf::RenderWindow* window, sf::Vector2f direction, sf::Vector2f origin, sf::Color color);

	std::vector<std::shared_ptr<Robot>> m_robots;

	sf::Texture m_texture;

	bool m_paused;

	bool m_viewLocked;

	std::vector<sf::Vector2f> m_arrowDirections;

	std::vector<sf::Vector2f> m_arrowOrigins;

	std::vector<sf::Color> m_arrowColors;

};


#endif