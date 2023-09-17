#ifndef MODULE_HPP
#define MODULE_HPP

#include "Config.hpp"

class Face;

class Robot;

class Module : public std::enable_shared_from_this<Module>
{
public:

	Module(sf::Vector2i position, int pressureID, std::shared_ptr<Robot> robot, sf::Texture& texture);

	sf::Vector2i GetPosition() {return m_position;}

	void SetUpFaces(int faceID1, int faceID2, int faceID3, int faceID4);

	sf::Vector2f GetCoMRelativePosition() {return m_CoMRelativePosition;}

	std::weak_ptr<Robot> GetRobot() {return m_robot;}

	void Draw(sf::RenderWindow* window);

	void SelectFace(sf::Vector2f position, int command);

	bool IsAnyFaceFiring();

	bool IsFaceFiring(int face);

	void UpdateFlows();

	void SetZeroFlow();

	void TrySetFlow(int flowID, float flow);

	void TrySetPressure(int pressureID, float pressure);

	void SetFlowCheckedAll(bool checked);

	void SetFlowChecked(int face, bool checked);

	bool GetFlowChecked(int face);

	sf::Vector2f GetLinearForce();
	float GetAngularForce();
	std::weak_ptr<Face> GetFace(int face) {return m_faces[face];}
	//std::shared_ptr<Face> GetFace(short ID);
	int GetFaceFlowID(int face);
	bool GetFaceBlocked(int face);
	int GetNumFacesBlocked();

	float GetPressure() {return m_pressure;}

	int GetPressureID() {return m_pressureID;}

	bool HasNeighbour(int neighbour);
	std::weak_ptr<Module> GetNeighbour(int neighbour);

private:

	sf::Vector2i m_position;

	sf::Vector2f m_CoMRelativePosition;

	std::vector<std::shared_ptr<Face>> m_faces;

	std::weak_ptr<Robot> m_robot;

	std::vector<sf::Sprite> m_pumpSprites;

	std::vector<sf::Sprite> m_waterSprites;

	int m_numFiringFaces;

	int m_numFiringFacesInto;

	//This is the module/'voltage' ID
	int m_pressureID;

	float m_pressure;

};

#endif