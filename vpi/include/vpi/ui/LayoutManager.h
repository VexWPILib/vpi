// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// TODO - Figure out how to get the callbacks to work properly

#if 0

#pragma once

extern vex::brain Brain;

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <vector>

#include "vex.h"

#include "vpi/ui/Button.h"

namespace vpi {

extern Button globalButtons[];
class LayoutManager {
  public:
    LayoutManager(vex::brain::lcd &screen) 
      : m_screen(screen)
    { }

    int AddButton(int x, int y, int w, int h, const char *label,
          vex::color uColor, vex::color pColor)
    {
      if(m_buttonIndex > 30) {
        return -1;
      }
      //Button *newButton = new Button(x, y, w, h, label, uColor, pColor);
      globalButtons[m_buttonIndex] = Button(x, y, w, h, label, "SELECTED", uColor, pColor, vex::color::blue);
      globalButtons[m_buttonIndex].SetOnPressed(myMarco);
      globalButtons[m_buttonIndex].SetOnReleased(myPolo);
      m_buttons.push_back(globalButtons[m_buttonIndex]);
      m_buttonIndex++;
    }

    void Draw() {
      m_screen.clearScreen( vex::color::black );
      for(Button b : m_buttons) {
        b.Draw(m_screen);
      }
    }

    void userTouchCallbackPressed() {
      Button *b = FindButtonPressed();
      if(b != NULL) {
        printf("Button hit\n");
        b->OnPressed();
        //b->Draw(m_screen);
        Draw();
      }
    }

    void userTouchCallbackReleased() {
      Button *b = FindButtonPressed();
      if(b != NULL) {
        printf("Button released\n");
        b->OnReleased();
        //b->Draw(m_screen);
        Draw();
      }
    }

  protected:
    vex::brain::lcd &m_screen;
    std::vector<Button> m_buttons;

    Button *FindButtonPressed() {
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();
      for(Button b : m_buttons) {
        if(b.IsInside(x, y)) {
          return &b;
        }
      }
      return NULL;
    }
  private:
    int m_buttonIndex=0;
}; // class Button

} // namespace vpi
#endif