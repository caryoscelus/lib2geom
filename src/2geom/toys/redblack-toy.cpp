/*
 * Copyright 2009  Evangelos Katsikaros <vkatsikaros at yahoo dot gr>
 *
 * This library is free software; you can redistribute it and/or
 * modify it either under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation
 * (the "LGPL") or, at your option, under the terms of the Mozilla
 * Public License Version 1.1 (the "MPL"). If you do not alter this
 * notice, a recipient may use your version of this file under either
 * the MPL or the LGPL.
 *
 * You should have received a copy of the LGPL along with this library
 * in the file COPYING-LGPL-2.1; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * You should have received a copy of the MPL along with this library
 * in the file COPYING-MPL-1.1
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
 * OF ANY KIND, either express or implied. See the LGPL or the MPL for
 * the specific language governing rights and limitations.
 */

/*
 initial toy for redblack trees
*/


#include <2geom/redblacktree.h>

#include <2geom/toys/path-cairo.h>
#include <2geom/toys/toy-framework-2.h>

#include <time.h>
using std::vector;
using namespace Geom;
using namespace std;


void draw_redblack_tree(cairo_t* cr, Geom::RedBlack *x, int depth = 0) {

	// works like the RedBlack::print_tree() (inorder function)
    if( x != 0 ){
        draw_redblack_tree(cr, x->left, depth+1);

        //print line and depth on the key of the node
		Geom::Point text_point = Point( x->key(), 10 );
		char label[4];
		sprintf(label,"%d",depth); // instead of std::itoa(depth, label, 10); 

		cairo_set_source_rgba (cr, 1, 0, 1, 1);
        draw_text(cr, text_point, label);

		Geom::Point line_2nd_point = Point( x->key(), 500 );
		//draw_line(text_point, line_2nd_point);
		cairo_move_to(cr, text_point );
        cairo_line_to(cr, line_2nd_point);
		cairo_set_source_rgba (cr, 0.5, 1, 0, 0.25);
        cairo_stroke(cr);

		draw_redblack_tree(cr, x->right, depth+1);
    }
}



class RedBlackToy: public Toy 
{
    PointSetHandle handle_set;
	Geom::Point starting_point;
	Geom::Point ending_point;

	Geom::RedBlackTree rbtree_x;
	RedBlack* search_result;

	int alter_existing_rect;
	int add_new_rect;
	int enable_printing; // used for debug - it disables the tree printing

	Rect *rect_chosen;
	int mode;

    enum menu_item_t
    {
        INSERT = 0,
		DELETE,
		SEARCH,
        TOTAL_ITEMS // this one must be the last item
    };

    static const char* menu_items[TOTAL_ITEMS];
    static const char keys[TOTAL_ITEMS];

//    virtual void draw(cairo_t *cr, std::ostringstream *notify, int width, int height, bool save, std::ostringstream *timer_stream) {
    void draw(cairo_t *cr, std::ostringstream *notify, int width, int height, bool save, std::ostringstream *timer_stream) {
        cairo_set_line_width( cr, 1 );

		
		for( unsigned i=0; i<handle_set.pts.size(); i=i+2 ){
	        Rect r1( handle_set.pts[i], handle_set.pts[i+1] );

		    cairo_set_source_rgba( cr, 0.5, 1, 0, 1 );
		    cairo_set_line_width( cr, 0.3 );
		    cairo_rectangle( cr, r1 );
            // the value to insert in the redblacktree: bounding box and the shapeID
		}

//		*notify << last_action;
		Toy::draw( cr, notify, width, height, save,timer_stream );

		cairo_set_source_rgba( cr, 1.0, 0.0, 0.0, 1.0 );
        cairo_stroke( cr );

		Toy::draw( cr, notify, width, height, save,timer_stream );
		draw_redblack_tree( cr, rbtree_x.root );
    }        
    
    void mouse_pressed(GdkEventButton* e) {
		Toy::mouse_pressed(e);
		if(e->button == 1){		// left mouse button
			if( mode == 0 ){	// insert / alter
				if(!selected) {
					starting_point = Point(e->x, e->y);
					add_new_rect = 1;
				}
				else
				{
					alter_existing_rect = 1;
				}
			}
			else if( mode == 1 ){	//search
					search_result = find_rectangle_of_point( Point(e->x, e->y) );
					if(search_result  != 0){
						std::cout << "Search found rect: (" << search_result->data 
							<< ": " << search_result->key() << ", " << search_result->high() 
							<< " : " << search_result->subtree_max << ") " << std::endl;
					}
					else{
						std::cout << "Search found NO rect" << std::endl;
					}
/*
				if(selected){				
					search_result = find_rectangle_of_point( Point(e->x, e->y) );
					if(rect_chosen != 0){
						std::cout << "Search found rect: " << std::endl;
					}
					else{
						std::cout << "Search found NO rect" << std::endl;
					}
				}
				else{
					std::cout << "In order to search click on a rect" << std::endl;
				}
*/
			}
			else if( mode == 2) {	// delete
			}
		}
		else if(e->button == 2){	//middle button
		}
		else if(e->button == 3){	//right button
		}
    }

    virtual void mouse_released(GdkEventButton* e) {
		Toy::mouse_released(e);
		if( e->button == 1 ) { 		//left mouse button
			if( mode == 0) {	// insert / alter
				if( add_new_rect ){
					ending_point = Point(e->x, e->y);
					//REMEMBER: handle same point issue in index, not in application level
					handle_set.push_back(starting_point);
					handle_set.push_back(ending_point);
					insert_in_tree_the_last_rect();
					add_new_rect = 0;
				}
				else if( alter_existing_rect ){
					//TODO update rect (and tree)
					alter_existing_rect = 0;
				}
			}
			else if( mode == 1 ){	// search

			}
			else if( mode == 2) {	// delete

			}
		}
		else if(e->button == 2){	//middle button
			
		}
		else if(e->button == 3){	//right button

		}
    }


    void key_hit(GdkEventKey *e)
    {
        char choice = std::toupper(e->keyval);
        switch ( choice )
        {
            case 'A':
                mode = 0;
                break;
            case 'B':
                mode = 1;
                break;
            case 'C':
                mode = 2;
                break;
        }
        //redraw();
    }

	void insert_in_tree_the_last_rect(){
			unsigned i = handle_set.pts.size() - 2;
		    Rect r1(handle_set.pts[i], handle_set.pts[i+1]);
			// insert in X axis tree
			rbtree_x.insert(r1, i, 0);	
			rbtree_x.print_tree();
	};

	RedBlack* find_rectangle_of_point(Point p){
		Coord a = 20.0;		
		Coord b = 350.0;
		return rbtree_x.search( new Interval( a, b) );				
/*		for( unsigned i=0; i<handle_set.pts.size(); i=i+2 ){
			// under the assumption that the "upper" handle is chosen when 
			// selecting two handles that are on top of each other
			if(p == handle_set.pts[i]){ // TODO fix the 2nd handler
				std::cout << "found same point";
				Coord a = 50.0;		// TODO dimension X Axis here
				Coord b = 150.0;
				return rbtree_x.search( new Interval( a, b) );				
			}
		}
		return 0;
*/
	}



public:
    RedBlackToy(): alter_existing_rect(0), add_new_rect(0), enable_printing(1), rect_chosen(0), mode(0){
        if(handles.empty()) {
            handles.push_back(&handle_set);
        }
    }


};



int main(int argc, char **argv) {
	std::cout << "---------------------------------------------------------"<< std::endl;
    std::cout << "Let's play with the Red Black Tree! ONLY Insert works now!!!"<< std::endl;
    std::cout << " Key A: insert/alter mode                                   "<< std::endl;
    std::cout << " * Left click and drag on white area: create a rectangle"<< std::endl;
	std::cout << " * Left click and drag on handler: alter a rectangle"<< std::endl;
    std::cout << " Key B: search mode                                   "<< std::endl;
	std::cout << " * Left click on handler:  \"search\" for a rectangle"<< std::endl;
    std::cout << " Key C: delete mode                                   "<< std::endl;
	std::cout << " * Middle click on handler: delete for a rectangle"<< std::endl;
	std::cout << "---------------------------------------------------------"<< std::endl;
    init(argc, argv, new RedBlackToy);
    return 0;
}
	  	 
const char* RedBlackToy::menu_items[] =
{
    "Insert / Alter Rectangle",
    "Search Rectangle",
    "Delete Reactangle"
};

const char RedBlackToy::keys[] =
{
     'A', 'B', 'C'
};