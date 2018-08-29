#ifndef TESTPRINTS_PRINTER_H
#define TESTPRINTS_PRINTER_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>


class Printer
{
public:

    enum Alignment { LEFT, CENTER, RIGHT};

    enum Color
    {
        RED = 31,
        GREEN = 32,
        YELLOW = 33,
        BLUE = 34,
        PURPLE = 35,
        TURKIS = 36,
        WHITE = 37,
        GREY = 38
    };

    Printer();

    void draw_top();
    void draw_mid();
    void draw_bottom();

    void write(std::string text, Color color = GREY, Alignment alignment = LEFT);

    std::string color_text(std::string text, Color color);

    void newline();

private:
    const char* right_bottom_corner = "\u2518";
    const char* left_bottom_corner = "\u2514";
    const char* right_top_corner = "\u2510";
    const char* left_top_corner = "\u250C";
    const char* right_separator = "\u2524";
    const char* left_separator = "\u251C";
    const char* horizontal_bar = "\u2500";
    const char* vertical_bar = "\u2502";
    const std::string block = "\u2588";

    const unsigned int WIDTH = 80;
    int cursor_index = 0;
    int space_left() {
        return WIDTH - cursor_index - 2;
    }

    void write_at_beginning(std::string text, Color color = GREY);
    void write_center(std::string text, Color color = GREY);
    void write_right(std::string text, Color color);



};


#endif //TESTPRINTS_PRINTER_H
