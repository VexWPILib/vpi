// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/ui/Graph.h"

using namespace vpi;

// Based on https://www.vexforum.com/t/graphing-on-the-v5-screen/78221

// Draw graph X and Y axis
// modify to fit your needs
void Graph::DrawAxis() {
    m_screen.setPenColor( vex::white );
    m_screen.drawLine( m_topLeftX, m_topLeftY, m_topLeftX, m_topLeftY + m_graphHeight );
    m_screen.drawLine( m_topLeftX, m_topLeftY, m_topLeftX + m_graphWidth, m_topLeftY );
    for( int x=0;x<m_graphWidth;x+=20 ) {
        m_screen.drawLine( m_topLeftX+x, m_topLeftY + m_graphHeight+5, m_topLeftX+x, m_topLeftY + m_graphHeight-5 );
    }
    for( int y=0;y<m_graphHeight;y+=20 ) {
        m_screen.drawLine( m_topLeftX+5, y, m_topLeftX-5, y );
    }
}

// draw everything
void Graph::Draw() {
    if(m_topLeftX == 0 && m_topLeftY == 0 && m_graphWidth == 480 && m_graphHeight == 240) {
        m_screen.clearScreen( vex::color(0x202020) );
    } else {
        m_screen.drawRectangle( m_topLeftX, m_topLeftY, m_graphWidth, m_graphHeight, vex::color(0x202020));
    }
    DrawAxis();
    for(int id=0;id<m_points.size();id++) {
        m_points[id]->Draw(m_topLeftX, m_topLeftY, m_graphHeight);
    }
    m_screen.render();
}