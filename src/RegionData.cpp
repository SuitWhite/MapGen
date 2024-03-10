#include "RegionData.h"
#include "VoronoiDiagram.h"
#include "Vector2.h"

#include <iostream>

void RegionData::calculateConvex() { 
    Vector2 center = site->point;

    Face* face = site->face;
    HalfEdge* halfEdge = face->outerComponent;
 
    if (halfEdge == nullptr)
        return;
    while (halfEdge->prev != nullptr)
    {
        halfEdge = halfEdge->prev;
        if (halfEdge == face->outerComponent)
            break;
    }
    HalfEdge* start = halfEdge;

    int number = 0;

    while (halfEdge != nullptr)
    {
        if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
        {
            number++;
        }

        halfEdge = halfEdge->next;
        if (halfEdge == start)
            break;
    }

    convex.setPointCount(number);
    convex.setFillColor(sf::Color(0, 204, 204));

    int inx = 0;

    while (halfEdge != nullptr)
    {
        if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
        {
            Vector2 destination = halfEdge->destination->point;
            convex.setPoint(inx, sf::Vector2f(destination.x, 1-destination.y));
            inx++;
        }

        halfEdge = halfEdge->next;
        if (halfEdge == start) {
            break;
        }
    }
}

Site *RegionData::getSite()
{
    return site;
}

RegionData::RegionData(Site *s, int h = 0, int r = 0){
    site = s;
    calculateConvex();
    setHeight(h);
    setRiver(r);
}

void RegionData::setHeight(int h) {
    calculateConvex();

    if (h < 0)
        h = 0;

    height = h; 
    if (h > 1)
        convex.setFillColor(sf::Color(102 - h / 2, 204 - h / 2, 0));
    else if ( h == 1 ) 
        convex.setFillColor(sf::Color(204, 204, 0));
    else
        convex.setFillColor(sf::Color(0, 204, 204));

    if (river > 0)
        convex.setFillColor(sf::Color::Cyan);
}

int RegionData::getHeight() {
    return height;
}

void RegionData::setRiver(int r) {
    river = r;
    if (river > 0)
        convex.setFillColor(sf::Color::Cyan);
}

int RegionData::getRiver() {
    return river;
}

sf::ConvexShape RegionData::getConvex() const {
    return convex;
}
