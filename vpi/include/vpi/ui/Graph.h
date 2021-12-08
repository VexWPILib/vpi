// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) James Pearman
// See: https://www.vexforum.com/t/graphing-on-the-v5-screen/78221

// TODO - Callbacks are not working on buttons

#if 0

#include "vex.h"
#include <vector>

// define your global instances of motors and other devices here
class graph {
  #define NUM_POINTS  480
  
  private:
    // class to hold points for a single graph line
    class points {
      public:
        uint32_t         *_points;
        vex::brain::lcd  &_screen;
        vex::color        _color;
      
        points( vex::brain::lcd &screen ) : _screen(screen) {
          // allocate memory on heap
          _points = new uint32_t[NUM_POINTS];
          // init everything to some value we consider invalid
          for(int i=0;i<NUM_POINTS;i++) {
            _points[i] = INT32_MAX;
          }
          // default line color
          _color = vex::white;
        }
        ~points(){
          // deallocate memory
          delete _points;
        }

        // draw the line
        // There's a variety of ways to do this, could be another property of the class
        void draw() {
          _screen.setPenColor( _color );
          for(int x=0;x<NUM_POINTS-2;x++) {
            if( _points[x] != INT32_MAX ) {
              _screen.drawLine( x, _points[x], x+1, _points[x+1]);
              _screen.drawCircle( x, _points[x], 2, _color );
            }
          }
        }

        // add a point to this line
        void addPoint( int value ) {
          for(int i=0;i<NUM_POINTS-1;i++) {
            _points[i] = _points[i+1];
          }
          _points[NUM_POINTS-1 ] = value;
        }

        // set color for this line
        void setColor( vex::color c ) {
          _color = c;
        }
    };
    
    public:
      vex::brain _brain;
      std::vector<graph::points *> _points;
      int   _origin_x;
      int   _origin_y;
      
      graph( int seqnum, int origin_x, int origin_y ) : _origin_x(origin_x), _origin_y(origin_y) {
        // allocate and store each line
        for( int i=0;i<seqnum;i++ ) {
          _points.push_back( new graph::points(_brain.Screen) );
        }

        // thread to render this graph
        thread( render, static_cast<void *>(this) );
      }
      ~graph(){
        // we should deallocate the vector members here really
      }

      // Thread that constantly draws all lines
      static int render(void *arg ) {
        if( arg == NULL)
          return(0);

        graph *instance = static_cast<graph *>(arg);

        while( 1) {
            // this will call render, no need for any other delays
            instance->draw();
        }

        return(0);
      }

      // Draw graph X and Y axis
      // modify to fit your needs
      void drawAxis() {
        _brain.Screen.setPenColor( vex::white );
        _brain.Screen.drawLine( _origin_x, 0, _origin_x, 240 );
        _brain.Screen.drawLine( 0, _origin_y, 480, _origin_y );
        for( int x=0;x<480;x+=20 ) {
          _brain.Screen.drawLine( x, _origin_y+5, x, _origin_y-5 );
        }
        for( int y=0;y<240;y+=20 ) {
          _brain.Screen.drawLine( _origin_x+5, y, _origin_x-5, y );
        }
      }

      // draw everything
      void draw() {
        _brain.Screen.clearScreen( vex::color(0x202020) );
        drawAxis();
        for(int id=0;id<_points.size();id++)
          _points[id]->draw();
        _brain.Screen.render();
      }

      // add a point to a particular sequence
      void addPoint( int id, int value ) {
        if( id < _points.size() )
          _points[id]->addPoint(value + _origin_y );
      }
      
      // set the color of this sequence
      void setColor( int id, vex::color c ) {
        if( id < _points.size() )
          _points[id]->setColor( c );
      }
};
#endif