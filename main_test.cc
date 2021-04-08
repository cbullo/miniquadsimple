#include <cmath>

void Solve2BarFK(float &xp, float &yp, float la, float lb, float theta1,
                 float theta2) {
  xp = la * cosf(theta1) + lb * cosf(theta1 + theta2);
  yp = la * sinf(theta1) + lb * sinf(theta1 + theta2);
}

bool Solve2BarIK(float xp, float yp, float la, float lb, float &theta1,
                 float &theta2, bool up_elbow) {
  float cos2 = (xp * xp + yp * yp - la * la - lb * lb) / (2.f * la * lb);
  float sin2 = sqrtf(1 - cos2 * cos2);
  if (up_elbow) {
    sin2 = -sin2;
  }
  theta2 = atan2(sin2, cos2);

  float k1 = la + lb * cos2;
  float k2 = lb * sin2;

  float sin1 = (yp * k1 - xp * k2) / (k1 * k1 + k2 * k2);
  float cos1 = (yp - k1 * sin1) / k2;

  theta1 = atan2(sin1, cos1);

  return true;
}

bool Solve5BarWithShift(float xp, float yp, float la, float lb1, float lb2,
                        float b1_b2_angle, float lc, float &theta1,
                        float &theta4, int dir) {
  float theta2;
  float theta3;
  float cx;
  float cy;

  // if (!Solve2BarIK(xp, yp, la, lb2, theta1, theta2, 0)) {
  //  return false;
  //}

  // Solve2BarFK(cx, cy, la, lb1, theta1, theta2 - b1_b2_angle);
  // if (!Solve2BarIK(cx - lc, cy, la, lb1, theta4, theta3, 1)) {
  //  return false;
  //}

  Solve2BarIK(xp + lc, yp, la, lb2, theta4, theta3, true);
  Solve2BarFK(cx, cy, la, lb1, theta4, theta3 + b1_b2_angle);
  Solve2BarIK(cx - lc, cy, la, lb1, theta1, theta2, false);

  // Serial.printf("theta1: %f, theta4: %f, xp: %f, yp: %f\r\n",
  //                theta1 - M_PI_2, theta4 - M_PI_2, xp, yp);

  return true;
}

struct LegConfig {
  float la;
  float lb1;
  float lb2;
  float b1_b2_angle;
  float lc;
  float ld;
};

LegConfig leg_config = {30.f, 34.f, 43.33f, (11.3f * M_PI) / 180.f, 13.f, 22.f};

#include <SFML/Graphics.hpp>

int main() {
  int ysize = 600;
  int xsize = 600;

  char c;

  float shift = 40.f;
  float xpr = 13.f + shift;
  float ypr = -40.f;

  float xpl = 13.f - shift;
  float ypl = -40.f;

  float la = leg_config.la;
  float lb1 = leg_config.lb1;
  float lb2 = leg_config.lb2;
  float cx = 0.f;
  float cy = 0.f;
  float b1_b2_angle = leg_config.b1_b2_angle;
  float lc = leg_config.lc;
  float ld = leg_config.ld;
  float theta1;
  float theta2;
  float theta3;
  float theta4;

  sf::RenderWindow window(sf::VideoMode(800, 800), "SFML works!");

  // let's define a view
  sf::View view(sf::FloatRect(-100.f, -100.f, 200.f, 200.f));
  // view.setViewport(sf::FloatRect(1.0f, 0.f, -1.f, 1.0f));
  view.setSize(200, -200);

  // activate it
  window.setView(view);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
      xpr = (sf::Mouse::getPosition(window).x / 800.f * 200.f) - 100.f;
      ypr = -((sf::Mouse::getPosition(window).y / 800.f * 200.f) - 100.f);
    }

    if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
      xpl = (sf::Mouse::getPosition(window).x / 800.f * 200.f) - 100.f;
      ypl = -((sf::Mouse::getPosition(window).y / 800.f * 200.f) - 100.f);
    }

    window.clear();

    {
      Solve2BarIK(xpl + shift, ypl, la, lb2, theta4, theta3, false);
      Solve2BarFK(cx, cy, la, lb1, theta4, theta3 + b1_b2_angle);
      Solve2BarIK(cx - lc, cy, la, lb1, theta1, theta2, true);

      float x10 = -shift;
      float y10 = 0.f;

      float x11 = x10 + la * cosf(theta4);
      float y11 = y10 + la * sinf(theta4);

      float x12 = x11 + lb1 * cosf(theta4 + theta3 + b1_b2_angle);
      float y12 = y11 + lb1 * sinf(theta4 + theta3 + b1_b2_angle);

      float x13 = x11 + lb2 * cosf(theta4 + theta3);
      float y13 = y11 + lb2 * sinf(theta4 + theta3);

      float x20 = lc - shift;
      float y20 = 0.f;

      float x21 = x20 + la * cosf(theta1);
      float y21 = y20 + la * sinf(theta1);

      float x22 = x21 + lb1 * cosf(theta1 + theta2);
      float y22 = y21 + lb1 * sinf(theta1 + theta2);

      sf::Vertex xline[] = {sf::Vertex(sf::Vector2f(0, 0)),
                            sf::Vertex(sf::Vector2f(10, 0))};

      sf::Vertex yline[] = {sf::Vertex(sf::Vector2f(0, 0)),
                            sf::Vertex(sf::Vector2f(0, 10))};
      window.draw(xline, 2, sf::LineStrip);
      window.draw(yline, 2, sf::LineStrip);

      sf::Vertex line1[] = {
          sf::Vertex(sf::Vector2f(x10, y10), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x11, y11), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x12, y12), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x13, y13), sf::Color::Blue)};
      window.draw(line1, 4, sf::LineStrip);

      sf::Vertex line2[] = {
          sf::Vertex(sf::Vector2f(x20, y20), sf::Color::Green),
          sf::Vertex(sf::Vector2f(x21, y21), sf::Color::Green),
          sf::Vertex(sf::Vector2f(x22, y22), sf::Color::Green)};
      window.draw(line2, 3, sf::LineStrip);

      sf::Vertex point(sf::Vector2f(xpr, ypr));
      point.color = sf::Color::Red;
      window.draw(&point, 1, sf::Points);
    }

    {
      Solve2BarIK(xpr - lc - shift, ypr, la, lb2, theta4, theta3, true);
      Solve2BarFK(cx, cy, la, lb1, theta4, theta3 - b1_b2_angle);
      Solve2BarIK(cx + lc, cy, la, lb1, theta1, theta2, false);

      float x10 = lc + shift;
      float y10 = 0.f;

      float x11 = x10 + la * cosf(theta4);
      float y11 = y10 + la * sinf(theta4);

      float x12 = x11 + lb1 * cosf(theta4 + theta3 - b1_b2_angle);
      float y12 = y11 + lb1 * sinf(theta4 + theta3 - b1_b2_angle);

      float x13 = x11 + lb2 * cosf(theta4 + theta3);
      float y13 = y11 + lb2 * sinf(theta4 + theta3);

      float x20 = 0.f + shift;
      float y20 = 0.f;

      float x21 = x20 + la * cosf(theta1);
      float y21 = y20 + la * sinf(theta1);

      float x22 = x21 + lb1 * cosf(theta1 + theta2);
      float y22 = y21 + lb1 * sinf(theta1 + theta2);

      sf::Vertex xline[] = {sf::Vertex(sf::Vector2f(0, 0)),
                            sf::Vertex(sf::Vector2f(10, 0))};

      sf::Vertex yline[] = {sf::Vertex(sf::Vector2f(0, 0)),
                            sf::Vertex(sf::Vector2f(0, 10))};
      window.draw(xline, 2, sf::LineStrip);
      window.draw(yline, 2, sf::LineStrip);

      sf::Vertex line1[] = {
          sf::Vertex(sf::Vector2f(x10, y10), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x11, y11), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x12, y12), sf::Color::Blue),
          sf::Vertex(sf::Vector2f(x13, y13), sf::Color::Blue)};
      window.draw(line1, 4, sf::LineStrip);

      sf::Vertex line2[] = {
          sf::Vertex(sf::Vector2f(x20, y20), sf::Color::Green),
          sf::Vertex(sf::Vector2f(x21, y21), sf::Color::Green),
          sf::Vertex(sf::Vector2f(x22, y22), sf::Color::Green)};
      window.draw(line2, 3, sf::LineStrip);

      sf::Vertex point(sf::Vector2f(xpr, ypr));
      point.color = sf::Color::Red;
      window.draw(&point, 1, sf::Points);
    }

    window.display();
  }

  return 0;
}