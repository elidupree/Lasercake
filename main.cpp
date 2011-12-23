
/* Simple program:  Create a blank window, wait for keypress, quit.

   Please see the SDL documentation for details on using the SDL API:
   /Developer/Documentation/SDL/docs.html
*/
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SDL.h"
#include "GL/gl.h"
#include "GL/glu.h"

#include "water.cpp"
#include <iostream>

static SDL_Surface *gScreen;

static void initAttributes ()
{
    // Setup attributes we want for the OpenGL context
    
    int value;
    
    // Don't set color bit sizes (SDL_GL_RED_SIZE, etc)
    //    Mac OS X will always use 8-8-8-8 ARGB for 32-bit screens and
    //    5-5-5 RGB for 16-bit screens
    
    // Request a 16-bit depth buffer (without this, there is no depth buffer)
    value = 16;
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, value);
    
    
    // Request double-buffered OpenGL
    //     The fact that windows are double-buffered on Mac OS X has no effect
    //     on OpenGL double buffering.
    value = 1;
    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, value);
}

static void printAttributes ()
{
    // Print out attributes of the context we created
    int nAttr;
    int i;
    
    int  attr[] = { SDL_GL_RED_SIZE, SDL_GL_BLUE_SIZE, SDL_GL_GREEN_SIZE,
                    SDL_GL_ALPHA_SIZE, SDL_GL_BUFFER_SIZE, SDL_GL_DEPTH_SIZE };
                    
    const char *desc[] = { "Red size: %d bits\n", "Blue size: %d bits\n", "Green size: %d bits\n",
                     "Alpha size: %d bits\n", "Color buffer size: %d bits\n", 
                     "Depth bufer size: %d bits\n" };

    nAttr = sizeof(attr) / sizeof(int);
    
    for (i = 0; i < nAttr; i++) {
    
        int value;
        SDL_GL_GetAttribute ((SDL_GLattr)attr[i], &value);
        printf (desc[i], value);
    } 
}

static void createSurface (int fullscreen)
{
    Uint32 flags = 0;
    
    flags = SDL_OPENGL;
    if (fullscreen)
        flags |= SDL_FULLSCREEN;
    
    // Create window
    gScreen = SDL_SetVideoMode (640, 640, 0, flags);
    if (gScreen == NULL) {
		
        fprintf (stderr, "Couldn't set 640x640 OpenGL video mode: %s\n",
                 SDL_GetError());
		SDL_Quit();
		exit(2);
	}
}

static void mainLoop ()
{
    SDL_Event event;
    int done = 0;
    int frame = 0;
    int p_mode = 0;

	for (EACH_LOCATION(loc))
	{
	tiles[loc].contents = AIR;
	}
	for (EACH_LOCATION(loc))
	{
		if (loc.x >= 4 && loc.x <= 6 && loc.y >= 4 && loc.y <= 6 && loc.z >= 1/* && loc.z <= 6*/ )tiles[loc].contents = WATER; //* 4 / 5;
		//else if (loc.x == 5 && loc.y >= 5 && loc.z == 0) ;
		else if (loc.x >= 3 && loc.x <= 7 && loc.y >= 3 && loc.y <= 7 && loc.z >= 1)tiles[loc].contents = ROCK;
		else if ((loc.x == 1 || loc.x == 9 || loc.y == 1 || loc.y == 9) && loc.z < 3)tiles[loc].contents = ROCK;
		//if (loc.x >= 5 && loc.x <= 11 && loc.y >= 5 && loc.y <= 11 && loc.z < 1)tiles[loc].water.amount = precision_scale * 4 / 5;
		/*if (loc.z == 0 && loc.x >= 5) tiles[loc].is_solid_rock =true;
		else if (loc.z == 1 && loc.x >= 10) tiles[loc].is_solid_rock =true;
		else if (loc.z == 2 && loc.x >= 15) tiles[loc].is_solid_rock =true;
		else if (loc.x == 19)tiles[loc].water.amount = precision_scale;*/
		/*if (loc.z < 8 && loc.x > 10) tiles[loc].is_solid_rock =true;
		else if (loc.x >= 12 && loc.y <= 13 && loc.y >= 7) { tiles[loc].water.amount = precision_scale; }
		else if (loc.y != 10 && loc.x > 10) tiles[loc].is_solid_rock =true;
		else if (loc.z > 8 && loc.x > 10 && loc.x < 18) tiles[loc].is_solid_rock =true;
		else if (loc.x > 10) { tiles[loc].water.amount = precision_scale; }*/
		/*if (loc.z < 20 - loc.x) tiles[loc].is_solid_rock = true;
		else if (loc.z >= 15 && (20 - loc.x) >= 15) tiles[loc].water.amount = precision_scale;*/
		/*if (loc.x == 0)tiles[loc].water.amount = precision_scale;
		else if (loc.x == 1 && loc.z > 0) tiles[loc].is_solid_rock = true;
		else if (loc.x == 5) tiles[loc].is_solid_rock = true;
		else if (loc.x == 2 && (loc.z % 4) == 1) tiles[loc].is_solid_rock = true;
		else if (loc.x == 3 && (loc.z % 2) == 1) tiles[loc].is_solid_rock = true;
		else if (loc.x == 4 && (loc.z % 4) == 3) tiles[loc].is_solid_rock = true;*/
	}
    
    while ( !done ) {

		/* Check for events */
		while ( SDL_PollEvent (&event) ) {
			switch (event.type) {

				case SDL_MOUSEMOTION:
					break;
				case SDL_MOUSEBUTTONDOWN:
					break;
				case SDL_KEYDOWN:
					if(event.key.keysym.sym == SDLK_p) ++p_mode;
					if(event.key.keysym.sym != SDLK_ESCAPE)break;
				case SDL_QUIT:
					done = 1;
					break;
				default:
					break;
			}
		}
		if(p_mode == 1)continue;
		if(p_mode > 1)--p_mode;
		int before_drawing = SDL_GetTicks();
		
		glClear(GL_COLOR_BUFFER_BIT);
		
		//drawing code here
		frame += 1;
	glLoadIdentity();
	gluPerspective(80, 1, 1, 100);
	gluLookAt(5.0 + 20.0 * std::cos((double)frame / 40.0),5.0 + 20.0 * std::sin((double)frame / 40.0),15.0 + 5.0 * std::sin((double)frame / 60.0),5,5,5,0,0,1);
		int total_amount = 0;
		int max_pressure = 0;
		int total_force = 0;
		int max_velmgsq = 0;
	for (EACH_LOCATION(loc))
	{
		if(loc.z == 0){
		glColor4f(0.2,0.4,0.0,1.0);

		glBegin(GL_POLYGON);
			glVertex3f(loc.x, loc.y, loc.z);
			glVertex3f(loc.x + 1, loc.y, loc.z);
			glVertex3f(loc.x + 1, loc.y +1, loc.z);
			glVertex3f(loc.x, loc.y+1, loc.z);
		glEnd();
		}
	}
	for (EACH_LOCATION(loc))
	{
		if (tiles[loc].contents != AIR)
		{
		  if (tiles[loc].contents == ROCK) {
		glColor4f(0.5,0.0,0.0,0.5);
		}
		else {
		  glColor4f(0.0, 0.0, 1.0, 0.5);
		}
		glBegin(GL_POLYGON);
			glVertex3f(loc.x, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y +1, (double)loc.z + 0.5);
			glVertex3f(loc.x, loc.y+1, (double)loc.z + 0.5);
		glEnd();
		}
		#if 0
		if (local_amount < precision_scale / 100) continue;
//std::cerr << local_amount << "\n";

//std::cerr << tiles[loc].water.velocity_delay.x << " " << tiles[loc].water.velocity_delay.y << " " << tiles[loc].water.velocity_delay.z << " " << "\n";
		glColor4f(0.0,0.0,1.0,(double)local_amount / (double)(precision_scale*2));
		glBegin(GL_POLYGON);
			glVertex3f(loc.x, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y, (double)loc.z + 0.5);
			glVertex3f(loc.x + 1, loc.y +1, (double)loc.z + 0.5);
			glVertex3f(loc.x, loc.y+1, (double)loc.z + 0.5);
		glEnd();
		glColor4f(0.0,0.0,1.0,((double)local_amount + (double)precision_scale) / (double)(precision_scale*2));
		glBegin(GL_LINES);
			glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
			glVertex3f((double)loc.x+0.5+((double)tiles[loc].water.velocity_delay.x/(double)precision_scale), (double)loc.y+0.5+((double)tiles[loc].water.velocity_delay.y/(double)precision_scale), (double)loc.z + 0.5+((double)tiles[loc].water.velocity_delay.z/(double)precision_scale));
		glEnd();
		glColor4f(0.0,1.0,0.0,((double)local_amount + (double)precision_scale) / (double)(precision_scale*2));
		glBegin(GL_LINES);
			glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
			glVertex3f((double)loc.x+0.5+((double)tiles[loc].water.velocity.x/(double)precision_scale), (double)loc.y+0.5+((double)tiles[loc].water.velocity.y/(double)precision_scale), (double)loc.z + 0.5+((double)tiles[loc].water.velocity.z/(double)precision_scale));
		glEnd();
		glBegin(GL_LINES);
		glColor4f(0.8,0.8,0.0,((double)local_amount + (double)precision_scale) / (double)(precision_scale*2));
			for(EACH_DIRECTION(d))
			{
				const scalar_type force_we = (outpushing_velocity[loc.x][loc.y][loc.z][1+d.x][1+d.y][1+d.z]);
				//std::cerr<<force_we;
				glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
				glVertex3f((double)loc.x+0.5+((double)force_we*d.x/(double)precision_scale), (double)loc.y+0.5+((double)force_we*d.y/(double)precision_scale), (double)loc.z + 0.5+((double)force_we*d.z/(double)precision_scale));
			}
		glEnd();
		glColor4f(1.0,0.0,0.0,((double)local_amount + (double)precision_scale) / (double)(precision_scale*2));
		glBegin(GL_LINES);
			glVertex3f((double)loc.x+0.5, (double)loc.y+0.5, (double)loc.z + 0.5);
			glVertex3f((double)loc.x+0.5+((double)tiles[loc].water.pressure/(double)precision_scale), (double)loc.y+0.5, (double)loc.z + 0.5+((double)tiles[loc].water.pressure/(double)precision_scale));
		glEnd();
		max_pressure = std::max(max_pressure, (int)tiles[loc].water.pressure);
		max_velmgsq = std::max(max_velmgsq, (int)(tiles[loc].water.velocity.dot(tiles[loc].water.velocity)));
		total_force += tiles[loc].water.total_force;

	#endif
//std::cerr << total_amount << "---"<<max_pressure << "---"<<total_force << "\n";
	}
		glFinish();	
        SDL_GL_SwapBuffers();
		int before_processing = SDL_GetTicks();
			std::cerr << time << "\n";
		
		//doing stuff code here
		update_water();
		int after = SDL_GetTicks();
std::cerr << (after - before_processing) << ", " << (before_processing - before_drawing) << "\n";

//	SDL_Delay(50);
		
	}
}

int main(int argc, char *argv[])
{
	// Init SDL video subsystem
	if ( SDL_Init (SDL_INIT_VIDEO) < 0 ) {
		
        fprintf(stderr, "Couldn't initialize SDL: %s\n",
			SDL_GetError());
		exit(1);
	}

    // Set GL context attributes
    initAttributes ();
    
    // Create GL context
    createSurface (0);
    
    // Get GL context attributes
    printAttributes ();
    
    // Init GL state
	gluPerspective(90, 1, 1, 100);
	gluLookAt(20,20,20,0,0,0,0,0,1);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    
    // Draw, get events...
    mainLoop ();
    
    // Cleanup
	SDL_Quit();
	
    return 0;
}
