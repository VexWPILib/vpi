// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once
#include "vex.h"
#include <vector>
// Based on https://www.vexforum.com/t/graphing-on-the-v5-screen/78221

namespace vpi {
class Graph {
  
  private:
    // class to hold points for a single graph line
    class Points {
      public:
        uint32_t         *m_points;
        vex::brain::lcd  &m_screen;
        vex::color        m_color;
        int               m_minY;
        int               m_maxY;
        int               m_maxWidth;
      
        Points( vex::brain::lcd &screen, int maxWidth) : 
          m_screen(screen), m_maxWidth(maxWidth) {
          // allocate memory on heap
          m_points = new uint32_t[m_maxWidth];
          // init everything to some value we consider invalid
          for(int i=0;i<m_maxWidth;i++) {
            m_points[i] = INT32_MAX;
          }
          // default line color
          m_color = vex::white;
        }
        ~Points(){
          // deallocate memory
          delete m_points;
        }

        // draw the line
        // There's a variety of ways to do this, could be another property of the class
        void Draw(int xOffset, int yOffset, int graphHeight) {
          m_screen.setPenColor( m_color );
          for(int x=0;x<m_maxWidth-2;x++) {
            if( m_points[x] != INT32_MAX ) {
              // Suppose h=240, minY=-100, maxY=100
              //    p=100, means draw at 0
              //    p=0, means draw at 120
              //    p=-100, means draw at 240
              m_screen.drawLine( xOffset + x, yOffset + graphHeight - ((m_points[x] - m_minY) * graphHeight ) / (m_maxY - m_minY), 
                                xOffset+x+1, yOffset + graphHeight - ((m_points[x+1] - m_minY) * graphHeight) / (m_maxY - m_minY));
              m_screen.drawCircle( xOffset + x, yOffset + graphHeight - ((m_points[x] - m_minY) * graphHeight) / (m_maxY - m_minY), 2, m_color );
            }
          }
        }

        // add a point to this line
        void AddPoint( int value ) {
          for(int i=0;i<m_maxWidth-1;i++) {
            m_points[i] = m_points[i+1];
          }
          m_points[m_maxWidth-1 ] = value;
        }

        // set color for this line
        void SetColor( vex::color c ) {
          m_color = c;
        }
        void SetMinY( int y ) { m_minY = y; }
        void SetMaxY( int y ) { m_maxY = y; }
    };
    
    public:
      vex::brain::lcd  &m_screen;
      std::vector<Graph::Points *> m_points;
      int   m_topLeftX;
      int   m_topLeftY;
      int   m_graphWidth;
      int   m_graphHeight;
      
      Graph( vex::brain::lcd &screen, int seqnum, 
              int top_left_x, int top_left_y ,
              int graph_width, int graph_height ) : m_screen(screen), 
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

        // allocate and store each line
        for( int i=0;i<seqnum;i++ ) {
          m_points.push_back( new Graph::Points(m_screen, m_graphHeight) );
        }

        // thread to render this graph
        thread( Render, static_cast<void *>(this) );
      }
      ~Graph(){
        // we should deallocate the vector members here really
      }

      // Thread that constantly draws all lines
      static int Render(void *arg ) {
        if( arg == NULL)
          return(0);

        Graph *instance = static_cast<Graph *>(arg);

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
      void AddPoint( int id, int value ) {
        if( id < m_points.size() )
          m_points[id]->AddPoint(value);
      }
      
      // set the color of this sequence
      void SetColor( int id, vex::color c ) {
        if( id < m_points.size() )
          m_points[id]->SetColor( c );
      }
      void SetMinY( int id, int y ) {
        if( id < m_points.size() )
          m_points[id]->SetMinY( y );
      }

      void SetMaxY( int id, int y ) {
        if( id < m_points.size() )
          m_points[id]->SetMaxY( y );
      }

};

};