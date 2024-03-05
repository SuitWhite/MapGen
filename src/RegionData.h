#pragma once
#include <SFML/Graphics.hpp>

struct Site;

class RegionData {
    Site *site;
    sf::ConvexShape convex;
    int height;
    
    void calculateConvex();
public:
    RegionData(Site *s, int h);

    void setHeight(int h);
    int getHeight();
    sf::ConvexShape getConvex() const;
};
