/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include "BitmapDatabase.hpp"

Screen1ViewBase::Screen1ViewBase()
{
    image1.setXY(0, 0);
    image1.setBitmap(Bitmap(BITMAP_BLUE_OCEAN_AESTHETIC_BACKGROUND_WALLPAPER_800X480_ID));

    button1.setXY(389, 72);
    button1.setBitmaps(Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_ID), Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_PRESSED_ID));

    add(image1);
    add(button1);
}

void Screen1ViewBase::setupScreen()
{

}