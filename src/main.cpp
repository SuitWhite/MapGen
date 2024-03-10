/* FortuneAlgorithm
 * Copyright (C) 2018 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
// SFML
#include <SFML/Graphics.hpp>
// My includes
#include "FortuneAlgorithm.h"

constexpr float WINDOW_WIDTH = 600.0f;
constexpr float WINDOW_HEIGHT = 600.0f;
constexpr float POINT_RADIUS = 0.005f;
constexpr float OFFSET = 1.0f;

std::vector<Vector2> generatePoints(int nbPoints, int seed)
{
    std::cout << "seed: " << seed << '\n';
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution (0.0, 1.0);

    std::vector<Vector2> points;
    for (int i = 0; i < nbPoints; ++i)
        points.push_back(Vector2{distribution(generator), distribution(generator)});

    return points;
}

void drawPoint(sf::RenderWindow& window, Vector2 point, sf::Color color)
{
    sf::CircleShape shape(POINT_RADIUS);
    shape.setPosition(sf::Vector2f(point.x - POINT_RADIUS, 1 - point.y - POINT_RADIUS));
    shape.setFillColor(color);
    window.draw(shape);
}

void drawMouse(sf::RenderWindow& window)
{
    sf::CircleShape shape(POINT_RADIUS);

    sf::Vector2f mouse; 
    mouse.x = sf::Mouse::getPosition(window).x / ( WINDOW_WIDTH * 1.1f );
    mouse.y = sf::Mouse::getPosition(window).y / (WINDOW_HEIGHT * 1.1f);

    shape.setPosition(sf::Vector2f(mouse.x - POINT_RADIUS, mouse.y - POINT_RADIUS));
    shape.setFillColor(sf::Color::Red);
    window.draw(shape);
}

void drawEdge(sf::RenderWindow& window, Vector2 origin, Vector2 destination, sf::Color color)
{
    sf::Vertex line[] =
    {
        sf::Vertex(sf::Vector2f(origin.x, 1.0f - origin.y), color),
        sf::Vertex(sf::Vector2f(destination.x, 1.0f - destination.y), color)
    };
    window.draw(line, 2, sf::Lines);
}

void drawPoints(sf::RenderWindow& window, VoronoiDiagram& diagram)
{
    for (std::size_t i = 0; i < diagram.getNbSites(); ++i)
        drawPoint(window, diagram.getSite(i)->point, sf::Color(100, 250, 50));
}

void drawDiagram(sf::RenderWindow& window, VoronoiDiagram& diagram)
{
    for (std::size_t i = 0; i < diagram.getNbSites(); ++i)
    {
        const Site* site = diagram.getSite(i);
        Vector2 center = site->point;

        Face* face = site->face;
        HalfEdge* halfEdge = face->outerComponent;
        if (halfEdge == nullptr)
            continue;
        while (halfEdge->prev != nullptr)
        {
            halfEdge = halfEdge->prev;
            if (halfEdge == face->outerComponent)
                break;
        }
        HalfEdge* start = halfEdge;
 
        while (halfEdge != nullptr)
        {
            if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
            {
                Vector2 origin = (halfEdge->origin->point - center) * OFFSET + center;
                Vector2 destination = (halfEdge->destination->point - center) * OFFSET + center;
                //drawEdge(window, origin, destination, sf::Color::Cyan);
            }

            halfEdge = halfEdge->next;
            if (halfEdge == start) {
                break;
            }
        }

        sf::ConvexShape convex;
        convex = diagram.getData(i)->getConvex();
        window.draw(convex);


        // rivers
        // if (diagram.getData(i)->getRiver() > 0) {
            
        // }
        


        // mouse
        sf::Vector2f mouse; 
        mouse.x = sf::Mouse::getPosition(window).x / ( WINDOW_WIDTH);
        mouse.y = sf::Mouse::getPosition(window).y / (WINDOW_HEIGHT);

        drawMouse(window);

        //std::cout << mouse.x << std::endl;

        // check if the mouse is inside the shape
        // if (convex.getGlobalBounds().contains(mouse.x, mouse.y))
        // {
        //     int ikkh = 0;
        //     for (const auto& j : diagram.getNeighbors(i)) {
        //     window.draw(convex);
        //         drawEdge(window, diagram.getNeighborsEdges(i)[ikkh].origin->point, diagram.getNeighborsEdges(i)[ikkh].destination->point, sf::Color::Red);
        //         drawPoint(window, diagram.getSite(diagram.getNeighbors(i)[ikkh])->point, sf::Color::Red);
        //         ikkh++;
        //     }
        // }
    }
}

VoronoiDiagram generateRandomDiagram(std::size_t nbPoints, int seed)
{
    // Generate points
    std::vector<Vector2> points = generatePoints(nbPoints, seed);

    // Construct diagram
    FortuneAlgorithm algorithm(points);
    auto start = std::chrono::steady_clock::now();
    algorithm.construct();
    auto duration = std::chrono::steady_clock::now() - start;
    std::cout << "construction: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    // Bound the diagram
    start = std::chrono::steady_clock::now();
    algorithm.bound(Box{-0.05, -0.05, 1.05, 1.05}); // Take the bounding box slightly bigger than the intersection box
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "bounding: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    VoronoiDiagram diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    start = std::chrono::steady_clock::now();
    bool valid = diagram.intersect(Box{0.0, 0.0, 1.0, 1.0});
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "intersection: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    if (!valid)
        throw std::runtime_error("An error occured in the box intersection algorithm");

    return diagram;
}

VoronoiDiagram generateDiagram(const std::vector<Vector2>& points)
{
    // Construct diagram
    FortuneAlgorithm algorithm(points);
    auto start = std::chrono::steady_clock::now();
    algorithm.construct();
    auto duration = std::chrono::steady_clock::now() - start;
    std::cout << "construction: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    // Bound the diagram
    start = std::chrono::steady_clock::now();
    algorithm.bound(Box{-0.05, -0.05, 1.05, 1.05}); // Take the bounding box slightly bigger than the intersection box
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "bounding: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    VoronoiDiagram diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    start = std::chrono::steady_clock::now();
    bool valid = diagram.intersect(Box{0.0, 0.0, 1.0, 1.0});
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "intersection: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    if (!valid)
        throw std::runtime_error("An error occured in the box intersection algorithm");

    return diagram;
}

void drawTriangulation(sf::RenderWindow& window, VoronoiDiagram& diagram)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto origin = diagram.getSite(i)->point;
        for (const auto& j : diagram.getNeighbors(i))
        {
            auto destination = diagram.getSite(j)->point;
            drawEdge(window, origin, destination, sf::Color::Green);
        }
    }
}

void generateHeights(VoronoiDiagram& diagram, int seed)
{
    srand(seed);
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto center = diagram.getSite(i)->point;
        auto data = diagram.getData(i);
        // std::cout << center << std::endl;
        
        int h = 0;//35 + rand() % 15 - abs(center.x*100-50) - abs(center.y*100-50);
        // std::cout << h << std::endl;
        if (h<0)
            h = 0;
        diagram.getData(i)->setHeight(h);
    }    
}

void generateBeach(VoronoiDiagram& diagram)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto data = diagram.getData(i);
        for (const auto& j : diagram.getNeighbors(i))
        {
            auto nbData = diagram.getData(j);
            if (data->getHeight() != 0 && nbData->getHeight() == 0)
                data->setHeight(1);
        }
    }
}

void generateRivers(VoronoiDiagram& diagram, int seed)
{
    int i = 0, river_size = 0;
    int number_of_rivers = 5;
    RegionData *tmpData;

    srand(seed);

    for (auto tmp = 0; tmp < number_of_rivers; ++tmp)
    {
        i = rand()%diagram.getNbSites();
        river_size = rand()%10;

        auto center = diagram.getSite(i)->point;

        if (diagram.getData(i)->getHeight() <= 1) {
            tmp--;
            continue;
        }

        tmpData = diagram.getData(i);
        std::cout << "TEST TEST TEST" << std::endl;

        while (tmpData->getHeight() >= 1) {
            tmpData->setRiver(river_size);
            std::cout << "TEST TEST TEST 2" << std::endl;

            auto lowestNb = diagram.getData(diagram.getNeighbors(tmpData->getSite()->index)[0]);
            for (const auto& j : diagram.getNeighbors(tmpData->getSite()->index))
            {
                auto nbData = diagram.getData(j);
                if (nbData->getHeight() < lowestNb->getHeight() && !nbData->getRiver())
                    //std::cout << tmpData->getHeight() << " " << nbData->getHeight() << std::endl;
                    lowestNb = nbData;
            }
            tmpData = lowestNb;

        }

    }
}

void increaseHeight(sf::RenderWindow& window, VoronoiDiagram &diagram)
{
    sf::Vector2f mouse; 
    sf::ConvexShape convex;
    std::size_t i;
    mouse.x = sf::Mouse::getPosition(window).x / (WINDOW_WIDTH);
    mouse.y = sf::Mouse::getPosition(window).y / (WINDOW_HEIGHT);

    for (i = 0; i < diagram.getNbSites(); ++i)
    {
        convex = diagram.getData(i)->getConvex();
        if (convex.getGlobalBounds().contains(mouse.x, mouse.y)) {
            diagram.getData(i)->setHeight(diagram.getData(i)->getHeight() + 5);

            for (const auto& j : diagram.getNeighbors(i)) {
                diagram.getData(j)->setHeight(diagram.getData(j)->getHeight() + 2);
            }

            generateBeach(diagram);
        }
    }  
}

void decreaseHeight(sf::RenderWindow& window, VoronoiDiagram &diagram)
{
    sf::Vector2f mouse; 
    sf::ConvexShape convex;
    std::size_t i;

    mouse.x = sf::Mouse::getPosition(window).x / (WINDOW_WIDTH);
    mouse.y = sf::Mouse::getPosition(window).y / (WINDOW_HEIGHT);

    for (i = 0; i < diagram.getNbSites(); ++i)
    {
        convex = diagram.getData(i)->getConvex();
        if (convex.getGlobalBounds().contains(mouse.x, mouse.y)) {
            diagram.getData(i)->setHeight(diagram.getData(i)->getHeight() - 5);

            for (const auto& j : diagram.getNeighbors(i)) {
                diagram.getData(j)->setHeight(diagram.getData(j)->getHeight() - 2);
            }

            generateBeach(diagram);
        }
    }    
}

int main()
{
    std::size_t nbPoints = 2500;
    
    uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    
    VoronoiDiagram diagram = generateRandomDiagram(nbPoints, seed);
    diagram = generateDiagram(diagram.computeLloydRelaxation());
    //diagram = generateDiagram(diagram.computeLloydRelaxation());
    //diagram = generateDiagram(diagram.computeLloydRelaxation());
    diagram.computeTriangulation();
    generateHeights(diagram, seed);
    //generateRivers(diagram, seed);
    generateBeach(diagram);
    
    // Display the diagram
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Fortune's algorithm", sf::Style::Default, settings);
    window.setView(sf::View(sf::FloatRect(-0.1f, -0.1f, 1.2f, 1.2f)));

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Key::N) {
                uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
                diagram = generateRandomDiagram(nbPoints, seed);
                diagram.computeTriangulation();
                generateHeights(diagram, seed);
                //generateRivers(diagram, seed);
                generateBeach(diagram);
            }
            else if (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Key::R) {
                diagram = generateDiagram(diagram.computeLloydRelaxation());
                diagram.computeTriangulation();
                generateHeights(diagram, seed);
                //generateRivers(diagram, seed);
                generateBeach(diagram);
            }
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    increaseHeight(window, diagram);
                } else if (event.mouseButton.button == sf::Mouse::Right) {
                    decreaseHeight(window, diagram);
                }
            }
        }

        window.clear(sf::Color::Black);

        drawDiagram(window, diagram);
        //drawPoints(window, diagram);
        //drawTriangulation(window, diagram);

        window.display();
    }

    return 0;
}
