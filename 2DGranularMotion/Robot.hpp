#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Config.hpp"

class Environment;

class Module;

class Robot : public std::enable_shared_from_this<Robot>
{
public:

	Robot(std::map<std::pair<int, int>, int> bodyPlan, sf::Vector2f position, int rotation, std::shared_ptr<Environment> environment);

	void Draw(sf::RenderWindow* window);

	//Used to draw the robot centrally
	void DrawStatic(sf::RenderWindow* window);

	void Update();

	void SetUpModules(std::map<std::pair<int, int>, int> bodyPlan);

	void SelectModule(sf::Vector2f position, int command);

	bool CanFireExternalFace();

	void FireExternalFace();
	void StopFiringExternalFace();

	bool IsModuleHere(sf::Vector2i position);

	void CalculateFlowIDs();

	void CalculateNumFlowEquations();

	sf::Vector2f				GetCoM() {return m_CoM;}
	float						GetRotation() {return m_rotation;}
	sf::Vector2f				GetPosition() {return m_position;}
	float						GetCharacteristicSize() {return m_characteristicSize;}
	float						GetModuleFlowRate(sf::Vector2i position);
	short						GetModuleFlowDirection(sf::Vector2i position);
	float						GetAngularVelocity() {return m_angularVelocity;}
	sf::Vector2f				GetVelocity() {return m_velocity;}
	std::weak_ptr<Environment>	GetEnvironment() {return m_environment;}
	std::weak_ptr<Module>		GetModule(sf::Vector2i module) {return m_modules[std::make_pair(module.x, module.y)];}

	void SetSpriteFaceFiring(sf::Vector2f modulePosition, short direction);
	void SetSpriteFaceNotFiring(sf::Vector2f modulePosition, short direction);
	void SetSpriteFaceBlocked(sf::Vector2f modulePosition, short direction);
	void SetSpriteFaceUnblocked(sf::Vector2f modulePosition, short direction);
	void SetSpriteFlowRate(sf::Vector2f modulePosition, short direction, short flowDirection, float flowStrength);
	void SetUpdateFlows() {m_needToUpdateFlows = true;}

private:

	std::map<std::pair<int, int>, std::shared_ptr<Module>> m_modules;

	sf::Texture m_texture;

	sf::RenderTexture m_renderTexture;

	sf::Sprite m_sprite;

	void SetUpSprite(std::map<std::pair<int, int>, int> bodyPlan);

	void CalculateSizeAndMidpoint(std::map<std::pair<int, int>, int> bodyPlan);

	void UpdateFlows();

	//Biggest width and height of organism in real units
	sf::Vector2i m_realSize;

	sf::Vector2f m_realMidpoint;

	sf::Vector2f m_realCoM;

	//Organism size in modules
	sf::Vector2i m_size;

	sf::Vector2f m_midpoint;

	sf::Vector2f m_CoM;

	float m_characteristicSize;

	sf::Vector2f m_position;

	sf::Vector2f m_velocity;

	short m_mass;

	float m_momentOfInertia;

	float m_rotation;

	float m_angularVelocity;

	int m_numFlowEquations;

	//Pointer back to environment
	std::weak_ptr<Environment> m_environment;

	int m_numExternalFiringFaces;

	int m_numExternalFaces;

	bool m_needToUpdateFlows;
};

#endif