#include "Environment.hpp"

int main()
{
	//sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "2D Granular Motion");


	std::shared_ptr<sf::RenderWindow> window = std::shared_ptr<sf::RenderWindow>(new sf::RenderWindow(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "2D Granular Motion"));


	std::shared_ptr<Environment> environment = std::shared_ptr<Environment>(new Environment());

	environment->AddRobot(0, 2, 2);

	sf::Clock mainClock;

	mainClock.restart();

	while(true)
	{
		environment->HandleEvents(&(*window));

		environment->Update();

		environment->Draw(&(*window));		

		while (mainClock.getElapsedTime().asSeconds() < 1 / float(FPS))
		{
		}

		mainClock.restart();
	}
}
