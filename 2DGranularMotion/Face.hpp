#ifndef FACE_HPP
#define FACE_HPP

#include "Config.hpp"

class Module;

class Face
{
public:

	Face(int direction, int flowID, bool external, std::shared_ptr<Module> module);

	void SetFiring();
	void SetNotFiring();
	void SetBlocked();
	void SetUnblocked();
	void SetFlowRate(float rate);
	void SetFlowDirection(short direction) {m_flowDirection = direction;}
	void SetFlowChecked(bool flow) {m_flowChecked = flow;}

	bool GetFlowChecked() {return m_flowChecked;}
	bool GetFiring() {return m_firing;}
	bool GetExternal() {return m_external;}
	bool GetBlocked() {return m_blocked;}
	sf::Vector2i GetNormalVector() {return m_normalVector;}

	sf::Vector2f GetLinearForce();
	float GetAngularForce();
	int GetFlowID() {return m_flowID;}

private:

	int m_direction;
	sf::Vector2i m_normalVector;

	sf::Vector2f m_CoMRelativePosition;

	std::weak_ptr<Module> m_module;

	bool m_firing;
	bool m_external;
	bool m_blocked;

	float m_flowRate;
	short m_flowDirection;

	sf::Vector2f CalculateDragForce();

	float CalculateRotationalDragForce();

	sf::Vector2f CalculateMovementForce();

	//Used when calculating flow rates
	bool m_flowChecked;

	int m_flowID;
};

#endif