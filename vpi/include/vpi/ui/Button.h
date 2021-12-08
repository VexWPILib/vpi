// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// TODO - Figure out how to get the callbacks to work properly

#if 0

#pragma once

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "vex.h"

namespace vpi {

typedef enum e_ButtonState {
  UNSELECTED,
  PRESSED,
  SELECTED,
} ButtonState;

typedef enum e_ButtonEventType {
  ONPRESSED,
  ONRELEASED,
} ButtonEventType;

class Button {

  public:

    Button(const Button&) = default;
    Button& operator=(const Button&) = default;
    Button(Button&&) = default;
    Button& operator=(Button&&) = default;

    Button() {}
    
    Button(int x, int y, int w, int h, 
            const char *label, const char *labelSelected,
            vex::color uColor, vex::color pColor, vex::color sColor,
            ButtonState s=ButtonState::UNSELECTED, bool isSelectable=true) 
      : m_xpos(x), m_ypos(y), m_width(w), m_height(h), 
        m_label(label), m_labelSelected(labelSelected),
        m_unselectedColor(uColor), m_pushedColor(pColor), 
        m_selectedColor(sColor), m_state(s), m_prevState(s), m_isSelectable(isSelectable)
    {}

    Button(int x, int y, int w, int h, 
            const char *label,
            vex::color uColor, vex::color pColor) 
      : m_xpos(x), m_ypos(y), m_width(w), m_height(h), 
        m_label(label), m_labelSelected(label),
        m_unselectedColor(uColor), m_pushedColor(pColor), 
        m_selectedColor(uColor), m_state(ButtonState::UNSELECTED),
        m_prevState(ButtonState::UNSELECTED), m_isSelectable(false)
    {}

    bool IsInside(int x, int y) {
      if(x >= m_xpos && x <= m_xpos + m_width) {
        if(y >= m_ypos && y <= m_ypos + m_height) {
          return true;
        }
      }
      return false;
    }

    int X() {return m_xpos;}
    int Y() {return m_ypos;}
    int Width() {return m_width;}
    int Height() {return m_height;}
    const char *Label() {return m_label;}
    const char *LabelSelected() {return m_labelSelected;}
    ButtonState CurButtonState() {return m_state;}
    vex::color UnselectedColor() {return m_unselectedColor;}
    vex::color PushedColor() {return m_pushedColor;}
    vex::color SelectedColor() {return m_selectedColor;}
    bool IsSelectable() {return m_isSelectable;}

    void Draw(vex::brain::lcd &screen) {
      ButtonState curState = CurButtonState();
      printf("Button.Draw %d\n", curState);
      vex::color c;
      const char *text;
      if(curState == ButtonState::SELECTED) {
        c = SelectedColor();
        text = LabelSelected();
      } else if(curState == ButtonState::PRESSED) {
        c = PushedColor();
        text = Label();
      } else {
        c = UnselectedColor();
        text = Label();
      }
      screen.drawRectangle(X(), Y(), Width(), Height(), c);
      int mStringHeight = screen.getStringHeight(text);
      int mStringWidth = screen.getStringWidth(text);
      int hOffset = Height() / 2 + mStringHeight / 2;
      int wOffset = Width() / 2 - mStringWidth / 2;
      screen.printAt(X() + wOffset, Y() + hOffset, false, text);
    }

    void SetOnPressed(void(*callback)(void)) {m_onPressed = callback;m_hasOnPressed=true;}
    void SetOnReleased(void(*callback)(void)) {m_onReleased = callback;m_hasOnReleased=true;}
    void OnPressed() {
      if(m_state != ButtonState::PRESSED) {
        m_prevState = m_state;
      }
      m_state = ButtonState::PRESSED;
      m_label = "PRESSED";
      m_xpos += 50;
      printf("Hello %d\n",m_xpos);
      if(m_hasOnPressed && m_onPressed != NULL) {
        printf("Has m_onPressed\n");
        m_onPressed();
      } else {
        printf("No m_onPressed\n");
      }
    }

    void OnReleased() {
      if(m_isSelectable) {
        if(m_prevState == ButtonState::UNSELECTED) {
          m_state = ButtonState::SELECTED;
        } else {
          m_state = ButtonState::UNSELECTED;
        }
      } else {
        m_state = ButtonState::UNSELECTED;
      }
      printf("World\n");

      m_label = "RELEASED";
      if(m_hasOnReleased && m_onReleased != NULL) {
        //m_onReleased();
        printf("Has m_onReleased\n");
      } else {
        printf("No m_onReleased\n");
      }
    }

  protected:
    int m_xpos = 0;
    int m_ypos = 0;
    int m_width = 0;
    int m_height = 0;
    const char *m_label = "";
    const char *m_labelSelected = "";
    ButtonState m_state = ButtonState::UNSELECTED;
    ButtonState m_prevState = ButtonState::UNSELECTED;
    vex::color m_unselectedColor = vex::color::red;
    vex::color m_pushedColor = vex::color::white;
    vex::color m_selectedColor = vex::color::blue;
    bool m_isSelectable = false;
    bool m_hasOnPressed = false;
    bool m_hasOnReleased = false;
    void (*m_onPressed)(void);
    void (*m_onReleased)(void);

}; // class Button

} // namespace vpi
#endif