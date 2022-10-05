// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once
#include "vex.h"
#include <vector>

#include "vpi/geometry/VexGpsPose2d.h"

namespace vpi {

class OdometryUI {
  
  private:
    class OdomDebugPoints {
      public:
        VexGpsPose2d     *m_points;
        vex::brain::lcd  &m_screen;
        int               m_minY;
        int               m_maxY;
        int               m_maxHistory;
        vex::color        m_firstColor;
        vex::color        m_middleColor;
        vex::color        m_lastColor;
        int               m_textOffset;

        OdomDebugPoints( vex::brain::lcd &screen, int maxHistory) :
          m_screen(screen), m_maxHistory(maxHistory) {
          // allocate memory on heap
          m_points = new VexGpsPose2d[m_maxHistory];
          // init everything to some value we consider invalid
          for(int i=0;i<m_maxHistory;i++) {
            m_points[i] = VexGpsPose2d(99_ft, 99_ft, 0_deg);
          }
          m_firstColor = vex::color::blue;
          m_middleColor = vex::color::yellow;
          m_lastColor = vex::color::red;
          m_textOffset = 0;
        }

        ~OdomDebugPoints(){
          // deallocate memory
          delete m_points;
        }

        void Draw(int xOffset, int yOffset, int graphDimPixels); 

        void AddPoint( VexGpsPose2d value ) {
          for(int i=0;i<m_maxHistory-1;i++) {
            m_points[i] = m_points[i+1];
          }
          m_points[m_maxHistory-1] = value;
        }

        void SetFirstColor(vex::color c) { m_firstColor = c; }
        void SetMiddleColor(vex::color c) { m_middleColor = c; }
        void SetLastColor(vex::color c) { m_lastColor = c; }
        void SetTextOffset(int offset) { m_textOffset = offset; }

    };

  public:
    vex::brain::lcd  &m_screen;
    OdometryUI::OdomDebugPoints m_points;
    int   m_topLeftX;
    int   m_topLeftY;
    int   m_graphWidth;
    int   m_graphHeight;
    int   m_fieldWidth;
    int   m_fieldHeight;
    
    OdometryUI( vex::brain::lcd &screen, 
            int top_left_x, int top_left_y ,
            int graph_width, int graph_height ) : m_screen(screen), m_points(screen, 20), 
                                                  m_topLeftX(top_left_x), m_topLeftY(top_left_y),
                                                  m_graphWidth(graph_width), m_graphHeight(graph_height) {
      if(m_topLeftX < 0) {
          m_topLeftX = 0;
      }
      if(m_topLeftY < 0) {
          m_topLeftY = 0;
      }
      if(m_graphWidth > 480) {
          m_graphWidth = 480;
      }
      if(m_graphHeight > 240) {
          m_graphHeight = 240;
      }
      m_fieldWidth = std::min(m_graphWidth, m_graphHeight);
      m_fieldHeight = std::min(m_graphWidth, m_graphHeight);

      // thread to render this graph
      thread( Render, static_cast<void *>(this) );
    }
    ~OdometryUI(){
      // we should deallocate the vector members here really
    }

    // Thread that constantly draws all lines
    static int Render(void *arg ) {
      if( arg == NULL)
        return(0);

      OdometryUI *instance = static_cast<OdometryUI *>(arg);

      while( 1) {
          // this will call render, no need for any other delays
          instance->Draw();
      }

      return(0);
    }

    // Draw graph X and Y axis
    // modify to fit your needs
    void DrawAxis();

    // draw everything
    void Draw();

    // add a point to a particular sequence
    void AddPoint( VexGpsPose2d value ) {
      m_points.AddPoint(value);
    }

    void SetFirstColor(vex::color c) { m_points.SetFirstColor(c); }
    void SetMiddleColor(vex::color c) { m_points.SetMiddleColor(c); }
    void SetLastColor(vex::color c) { m_points.SetLastColor(c); }
    void SetTextOffset(int offset) { m_points.SetLastColor(offset); }
};

};