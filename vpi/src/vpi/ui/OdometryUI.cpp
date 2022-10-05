// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/ui/OdometryUI.h"

using namespace vpi;

void  OdometryUI::OdomDebugPoints::Draw(int xOffset, int yOffset, int graphDimPixels) {
    bool firstPoint = true;
    bool lastPoint = false;
    int fieldDimensionInInches = 12 * 12; // 12 feet by 12 feet

    m_screen.setPenColor( m_lastColor );

    for(int i=0;i<m_maxHistory;i++) {
        if( m_points[i].X() != 99_ft ) {
            vex::color dataColor = m_middleColor;
            if(firstPoint) {
                dataColor = m_firstColor;
                firstPoint = false;
            }
            if(i == m_maxHistory-1) {
                lastPoint = true;
                dataColor = m_lastColor;
            }
            // Suppose h=240, x=6_ft, y=-6ft is the bottom-left corner (xOffset, yOffset+h)
            // Suppose h=240, x=6_ft, y=0ft is the left middle (xOffset, yOffset + h / 2)
            // Suppose h=240, x=6_ft, y=6ft is the top-left (xOffset, yOffset)
            int x = m_points[i].X().convert(inch) + fieldDimensionInInches / 2; // shift origin from field center to bottom-left corner
            int y = m_points[i].Y().convert(inch) + fieldDimensionInInches / 2; // shift origin from field center to bottom-left corner
            m_screen.drawCircle(xOffset + (x * graphDimPixels) / fieldDimensionInInches,
                                yOffset + graphDimPixels - (y * graphDimPixels / fieldDimensionInInches),
                                5, dataColor);
            if(lastPoint) {
                int textY = 20 + m_textOffset;
                if(m_points[i].Y().convert(inch) > 0) {
                    textY = graphDimPixels / 2 + 20 + m_textOffset;
                }
                m_screen.setPenColor( dataColor );
                m_screen.printAt(xOffset + 10,yOffset + textY,"(%d in,%d in) %d deg", 
                                            (int)m_points[i].X().convert(inch), 
                                            (int)m_points[i].Y().convert(inch), 
                                            (int)m_points[i].Theta().convert(degree));

            }
        }
    }
}

// Draw graph X and Y axis
// modify to fit your needs
void OdometryUI::DrawAxis() {

    // Suppose h=240, and w=480. We have a 12x12 square we want to draw
    // First, fit it into a square, done in constructor
    // Second, draw tiles
    m_screen.setPenColor( vex::color(0xC0C0C0C0) ); // Go for a gray for tile lines
    for( int x=0;x<m_fieldWidth;x+=m_fieldWidth/6 ) {
        m_screen.drawLine( m_topLeftX+x, m_topLeftY, m_topLeftX+x, m_topLeftY + m_fieldWidth );
    }
    for( int y=0;y<m_fieldHeight;y+=m_fieldHeight/6 ) {
        m_screen.drawLine( m_topLeftX, y, m_topLeftX + m_fieldHeight, y );
    }

    // Third, draw coordinate system
    m_screen.setPenColor( vex::white );
    m_screen.drawLine( m_topLeftX + m_fieldWidth / 2, m_topLeftY, m_topLeftX + m_fieldWidth / 2, m_topLeftY + m_fieldHeight );
    m_screen.drawLine( m_topLeftX, m_topLeftY + m_fieldHeight / 2, m_topLeftX + m_fieldWidth, m_topLeftY + m_fieldHeight / 2);

    // Fourth, draw Edges
    m_screen.setPenColor( vex::white );
    m_screen.drawLine( m_topLeftX + m_fieldWidth, m_topLeftY, m_topLeftX + m_fieldWidth, m_topLeftY + m_fieldHeight );
    m_screen.drawLine( m_topLeftX, m_topLeftY + m_fieldHeight, m_topLeftX + m_graphWidth, m_topLeftY + m_fieldHeight);

}

// draw everything
void OdometryUI::Draw() {
    if(m_topLeftX == 0 && m_topLeftY == 0 && m_graphWidth == 480 && m_graphHeight == 240) {
        m_screen.clearScreen( vex::color(0x202020) );
    } else {
        m_screen.drawRectangle( m_topLeftX, m_topLeftY, m_graphWidth, m_graphHeight, vex::color(0x202020));
    }
    DrawAxis();
    m_points.Draw(m_topLeftX, m_topLeftY, m_graphHeight);
    m_screen.render();
}