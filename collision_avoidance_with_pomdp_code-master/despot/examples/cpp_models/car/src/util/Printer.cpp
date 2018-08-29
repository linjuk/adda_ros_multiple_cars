#include <cmath>
#include <algorithm>
#include <iomanip>
#include "util/Printer.h"



void Printer::write(std::string text, const Color color, const Alignment alignment) {
    if (cursor_index == 0) {
        switch (alignment) {
            case LEFT:
                write_at_beginning(text, color);
                break;
            case CENTER:
                write_center(text, color);
                break;
            case RIGHT:
                write_right(text, color);
                break;
            default: break;
        }
        return;
    }
    if (text == block)
        cursor_index -=2;
    cursor_index += text.size();

    std::cout << color_text(text, color);
}

std::string Printer::color_text(const std::string text, const Printer::Color color) {
    std::string c = std::to_string(color);
    return "\033[" + c + 'm' + text + "\033[0m";
}

void Printer::draw_top() {
    std::cout << left_top_corner;
    for (int i = 0; i < space_left(); ++i) {
        std::cout << horizontal_bar;
    }
    std::cout << right_top_corner;
    std::cout << std::endl;
}

void Printer::draw_bottom() {
    if (cursor_index != 0) {
        newline();
    }

    std::cout << left_bottom_corner;
    for (int i = 0; i < space_left(); ++i) {
        std::cout << horizontal_bar;
    }
    std::cout << right_bottom_corner;
    std::cout << std::endl;

}

void Printer::write_at_beginning(std::string text, const Color color) {
    std::string colored_text = color_text(text, color);
    std::cout << vertical_bar << colored_text;
    cursor_index += text.length();
}

void Printer::draw_mid() {
    if (cursor_index != 0) {
        newline();
    }
    std::cout << left_separator;
    for (int i = 0; i < space_left(); ++i)
        std::cout << horizontal_bar;
    std::cout << right_separator << std::endl;
    cursor_index = 0;
}

void Printer::newline() {
    if (cursor_index == 0) {
        std::cout << vertical_bar;
    }
    for (int i = 0; i < space_left(); ++i) {
        std::cout << " ";
    }
    std::cout << vertical_bar << std::endl;
    cursor_index = 0;
}

void Printer::write_center(std::string text, Printer::Color color) {
    if (cursor_index != 0) {
        newline();
    }

    if (text.size() > WIDTH - 2) {
        long count = std::count(text.begin(), text.end(), ' ');
        unsigned long position = 0;
        if (count > 0) {
            position = text.find_first_of(' ', text.size() / 2);
        }
        std::string first_half = text.substr(0, position);
        std::string other_half = text.substr(position+1, text.size() - first_half.size());

        write_center(first_half, color);
        write_center(other_half, color);
        return;
    }

    unsigned long size = text.size();
    unsigned long left = (WIDTH - size - 2) / 2;
    unsigned long right = WIDTH - left - size - 2;

    std::cout << vertical_bar;
    for (int i = 0; i < left; ++i) {
        std::cout << " ";
    }
    std::cout << color_text(text, color);
    for (int i = 0; i < right; ++i) {
        std::cout << " ";
    }
    std::cout << vertical_bar << std::endl;
}

void Printer::write_right(std::string text, const Printer::Color color) {
    std::cout << vertical_bar;
    unsigned long size = text.size();
    for (int i = 0; i < space_left() - size; ++i) {
        std::cout << " ";
    }
    std::cout << color_text(text, color) << vertical_bar << std::endl;
    cursor_index = 0;
}

Printer::Printer() = default;

