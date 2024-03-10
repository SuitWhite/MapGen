#pragma once
#include <SFML/Graphics.hpp>

struct Site;

class RegionData {
    Site *site;
    sf::ConvexShape convex;
    int height;

    int river;
    
    void calculateConvex();
public:
    RegionData(Site *s, int h, int r);

    Site *getSite();

    void setHeight(int h);
    int getHeight();
    
    void setRiver(int r);
    int getRiver();
    
    sf::ConvexShape getConvex() const;
};
