#pragma once

class GJK {
public:
	bool Check(sf::Vector2f* ver1, uint16_t count1, sf::Vector2f* ver2, uint16_t count2);
	
	void Draw(sf::RenderWindow* m_window, bool ShowMinkSpace = true);
private:
	void AddVertexes(sf::Vector2f* ver, uint16_t count, sf::Color col);
	void CreateMinowskiSpace();
	sf::Vector2f Support(sf::Vector2f* ver1, uint16_t count1, sf::Vector2f* ver2, uint16_t count2, sf::Vector2f direction);
private:
	std::vector<sf::Vertex> m_Vertexes; //ONLY for rendering
	std::vector<sf::RectangleShape> m_MinkSpaceVer;

	sf::Vector2f* v1;
	sf::Vector2f* v2;
	uint16_t c1;
	uint16_t c2;

	int iter_count = 0;

	sf::Vector2f simplex[3];
};