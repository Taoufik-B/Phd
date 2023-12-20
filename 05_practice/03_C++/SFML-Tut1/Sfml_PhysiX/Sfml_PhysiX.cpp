// Sfml_PhysiX.cpp : Defines the entry point for the application.
//
#include <SFML/Graphics.hpp>
#include "Sfml_PhysiX.h"

using namespace std;


class GravitySource 
{

    sf::Vector2f pos;
    float strength;
    sf::CircleShape s;
    public:GravitySource(float pos_x, float pos_y, float strength) 
    {
        pos.x = pos_x;
        pos.y = pos_y;
        this->strength = strength;
        s.setPosition(pos);
        s.setFillColor(sf::Color::White);
        s.setRadius(10.f);
    }

    void render(sf::RenderWindow &window) 
    {
        window.draw(s);
    }

    sf::Vector2f get_pos()
    {
        return pos;
    }

    float get_strength()
    {
        return strength;
    }
};


class Particule 
{

    sf::Vector2f pos;
    sf::Vector2f vel;
    sf::CircleShape s;
    public:Particule(float pos_x, float pos_y, float vel_x, float vel_y)
    {
        pos.x = pos_x;
        pos.y = pos_y;
        vel.x = vel_x;
        vel.y = vel_y;
        s.setPosition(pos);
        s.setFillColor(sf::Color::White);
        s.setRadius(4.f);
    }

    void render(sf::RenderWindow& window)
    {
        s.setPosition(pos);
        window.draw(s);
    }

    void update_physics(GravitySource &s)
    {
        sf::Vector2f source_pos = s.get_pos();
        float distance_x = source_pos.x - pos.x;
        float distance_y = source_pos.y - pos.y;

        float distance = sqrt(distance_x * distance_x + distance_y * distance_y);


        float inverse_distance = 1.f / distance;

        float normalized_x = inverse_distance * distance_x;
        float normalized_y = inverse_distance * distance_y;

        float inverse_square_dropoff = inverse_distance * inverse_distance;
        float acc_x = normalized_x * s.get_strength() * inverse_square_dropoff;
        float acc_y = normalized_y * s.get_strength() * inverse_square_dropoff;

        vel.x += acc_x;
        vel.y += acc_y;

        pos.x += vel.x;
        pos.y += vel.y;

    }

};


class Bounce {

    sf::Vector2f pos;
    sf::Vector2f vel;
    sf::CircleShape s;
    public:Bounce(float pos_x, float pos_y, float vel_x, float vel_y)
    {
        pos.x = pos_x;
        pos.y = pos_y;
        vel.x = vel_x;
        vel.y = vel_y;
        s.setPosition(pos);
        s.setFillColor(sf::Color::Yellow);
        s.setRadius(4.f);
        s.setPointCount(4);
    }

      void render(sf::RenderWindow& window)
      {
          s.setPosition(pos);
          window.draw(s);
      }

      void update()
      {
          pos.x += vel.x;
          pos.y += vel.y;
      }
};

class Player
{
    sf::RectangleShape player;
    public:
    Player(sf::Vector2f size)
    {
        player.setSize(size);
        player.setFillColor(sf::Color::Green);
    }
    void draw(sf::RenderWindow& window)
    {
        window.draw(player);
    }
    void move(sf::Vector2f distance)
    {
        player.move(distance);
    }
    void setPosition(sf::Vector2f position)
    {
        player.setPosition(position);
    }
    int getY() {
        return player.getPosition().y;
    }

    sf::Vector2f getPosition()
    {
        return player.getPosition();
    }

    bool moveTo(sf::Vector2f position, float speed)
    {
        sf::Vector2f dOffset = position - player.getPosition();
        cout << dOffset.x <<" "<< dOffset.y << endl;
        if (abs(dOffset.x) <= speed && abs(dOffset.y) <= speed) 
        {

            return false;
        }
        else
        {
            player.move({ copysignf(speed, dOffset.x), copysignf(speed, dOffset.y) });
            return true;
        }
    }
};

int main()

{
    //sf::ContextSettings settings;
    //settings.antialiasingLevel = 8.0;
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML works!");
    //window.setFramerateLimit(60);

    GravitySource source(400, 300, 1000);

    //Particule particule(350, 250, 4, 0);

    //Bounce uav(200, 400, 2, 0);

    Player rawane({ 40, 40 });
    rawane.setPosition({ 300, 300 });


    // Gravity variables

    const int groundHeight = 500;
    const float gravitySpeed = 0.3;
    bool isJumping = false;
    bool isMoving = false;
    sf::Vector2f dOffset;

    while (window.isOpen())
    {
        sf::Event event;

        const float moveSpeed = 0.2;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) 
        {
            rawane.move({ 0, -moveSpeed });
            isJumping = true;
        }
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
        {
            rawane.move({ moveSpeed, 0 });
        }        
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
        {
            rawane.move({ -moveSpeed, 0 });
        }
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
        {
            rawane.move({ 0, moveSpeed });
        }
        else if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            isMoving = true;
            dOffset = sf::Vector2f(sf::Mouse::getPosition(window));
        }
        

        //cout << sf::Mouse::getPosition(window).x << endl;

        if (isMoving)
        {
            isMoving = rawane.moveTo(dOffset, 0.4F);
            cout << isMoving << endl;
        }

        while (window.pollEvent(event))
        {
            switch (event.type)
            {
                case sf::Event::Closed:
                    window.close();
                case sf::Event::KeyReleased:
                    isJumping = false; 
            }
        }

        //Gravity logic
        
        //if (rawane.getY() <= groundHeight && isJumping == false)
        //{
        //    rawane.move({ 0, gravitySpeed });
        //}

        window.clear();
        source.render(window);
        //particule.render(window);
        //
        //particule.update_physics(source);
        
        rawane.draw(window);
        //uav.render(window);
        //uav.update();
        window.display();
    }

    return 0;
}
