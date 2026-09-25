//Inclusions
#include "main.h"
#include "display.hpp"
#include "autons.hpp"
#include "liblvgl/lvgl.h"
#include "gif-pros/gifclass.hpp"
// #include "gradient.c"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "subsystems/drive.hpp"

//Definitions
#define DO_NOT_RUN 6104

//Sets up Variables
float* kP_ptr = nullptr;
float* kI_ptr = nullptr;
float* kD_ptr = nullptr;
int selected_program = DO_NOT_RUN;
int auto_type = 0;
int selected_profile = 0;
int selected_GIF = 5;
int selected_debug_option = 0;
char kP_char[50];
char kI_char[50];
char kD_char[50];
static bool btnmatrix_kP_first = true;
static bool btnmatrix_kI_first = true;
static bool btnmatrix_kD_first = true;
static bool pressed_kP = false;
static bool pressed_kI = false;
static bool pressed_kD = false;
static const char* TXT_HOLD;



//Makes an array of file directories to different gifs on the sd card
static std::vector<const char*> gif_file_paths = {
    "/usd/Sylvie.gif", "/usd/Catapult.gif", "/usd/Alliance.gif", "/usd/Glitch.gif", 
    "/usd/Lightning.gif", "/usd/Rumble.gif", "/usd/jellyblackoutglitch.gif", 
    "/usd/cryaboutit.gif", "/usd/casey.gif", "/usd/Harold.gif"
};

//Makes arrays for the different programs
static std::array<void(*)(), 10> red_scripts = {{
    right_red_one_auton, right_red_two_auton, red_solo_awp,
    right_red_elim_match_auton, disabled_ladder_auton, left_red_one_auton,
    left_red_two_auton, left_red_three_auton, left_red_elim_match_auton,
    disabled_auton
}};

static std::array<void(*)(), 10> blue_scripts = {{
    right_blue_one_auton, right_blue_two_auton, blue_solo_awp,
    right_blue_elim_match_auton, disabled_ladder_auton, left_blue_one_auton,
    left_blue_two_auton, left_blue_three_auton, left_blue_elim_match_auton,
    disabled_auton
}};

static std::array<void(*)(), 2> skills_scripts = {{
    programming_skills, driver_skills
}};





//Creates the LVGL Objects

//Creates Screens
static lv_obj_t* auton_screen = lv_obj_create(nullptr);
static lv_obj_t* skills_screen = lv_obj_create(nullptr);
static lv_obj_t* debug_screen = lv_obj_create(nullptr);
static lv_obj_t* debug_motor_screen = lv_obj_create(nullptr);
static lv_obj_t* debug_electronics_screen = lv_obj_create(nullptr);
static lv_obj_t* profile_screen = lv_obj_create(nullptr);
static lv_obj_t* visual_screen = lv_obj_create(nullptr);
static lv_obj_t* pid_screen = lv_obj_create(nullptr);

//Creates tabviews
lv_obj_t * motor_tabview = lv_tabview_create(debug_motor_screen, LV_DIR_LEFT, 80);

lv_obj_t * motor_tab_one = lv_tabview_add_tab(motor_tabview, "temp");
lv_obj_t * motor_tab_two = lv_tabview_add_tab(motor_tabview, "temp");
lv_obj_t * motor_tab_three = lv_tabview_add_tab(motor_tabview, "temp");
lv_obj_t * motor_tab_four = lv_tabview_add_tab(motor_tabview, "temp");
lv_obj_t * motor_tab_five = lv_tabview_add_tab(motor_tabview, "temp");
lv_obj_t * motor_tab_six = lv_tabview_add_tab(motor_tabview, "temp");

//Creates drowdown menus
lv_obj_t* ddl_color_selector = lv_dropdown_create(auton_screen);
lv_obj_t* ddl_auton_selector = lv_dropdown_create(auton_screen);
lv_obj_t* ddl_skills_selector = lv_dropdown_create(skills_screen);
lv_obj_t* ddl_profile_selector = lv_dropdown_create(profile_screen);
lv_obj_t* ddl_debug_option_selector = lv_dropdown_create(debug_screen);
lv_obj_t* ddl_GIF = lv_dropdown_create(visual_screen);
lv_obj_t* ddl_auton = lv_dropdown_create(auton_screen);
lv_obj_t* ddl_skills = lv_dropdown_create(skills_screen);
lv_obj_t* ddl_profile = lv_dropdown_create(profile_screen);
lv_obj_t* ddl_debug = lv_dropdown_create(debug_screen);
lv_obj_t* ddl_vision = lv_dropdown_create(visual_screen);

//Creates text areas
lv_obj_t* kP_text_area = lv_textarea_create(pid_screen);
lv_obj_t* kI_text_area = lv_textarea_create(pid_screen);
lv_obj_t* kD_text_area = lv_textarea_create(pid_screen);

//Creates Button Matrices
lv_obj_t* btnmatrix_kP = lv_btnmatrix_create(pid_screen);
lv_obj_t* btnmatrix_kI = lv_btnmatrix_create(pid_screen);
lv_obj_t* btnmatrix_kD = lv_btnmatrix_create(pid_screen);

//Creates Buttons
lv_obj_t* motor_info_screen_back_button = lv_btn_create(debug_motor_screen);

//Creates Arcs
lv_obj_t* drive_motor_temp_arc = lv_arc_create(debug_motor_screen);
lv_obj_t* speed_arc = lv_arc_create(debug_electronics_screen);


//gif
lv_obj_t* gif_obj_gif = lv_obj_create(visual_screen);
static Gif gif_preview = Gif("/usd/SPSIntro.gif", gif_obj_gif);



//Creates the action for when using the main dropdownlist
static void drop_down_menuing_action(lv_event_t* event) { 
    //Gets the lvgl object from the event
    lv_obj_t* dropdown = lv_event_get_target(event);
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //Loads different screens depending on the selected_index, as well as updating the other ddls to reflect this change
    switch (selected_index){
        
        case 0:
            // if first option selected loads auton screen and updates the ddl on the auton screen.
            lv_scr_load(auton_screen);
            lv_dropdown_set_selected(ddl_auton, 0);
            break;

        case 1:
            // if second option selected loads skills screen and updates the ddl on the skills screen.
            lv_scr_load(skills_screen);
            lv_dropdown_set_selected(ddl_skills, 1);
            break;

        case 2:
            // if third option selected loads profile screen and updates the ddl on the profile screen.
            lv_scr_load(profile_screen);
            lv_dropdown_set_selected(ddl_profile, 2);
            break;

        case 3:
        // if fourth option selected loads debug screen and updates the ddl on the debug screen.
            lv_scr_load(debug_screen);
            lv_dropdown_set_selected(ddl_debug, 3);
            break;

        case 4:
        // if fifth option selected loads Gif screen and updates the ddl on the gif screen.
            lv_scr_load(visual_screen);
            lv_dropdown_set_selected(ddl_vision,4);
            break;

        default:
            break;

    }
}


//Creates the action for when using the color selector ddl
static void dropdown_color_selection_action(lv_event_t * event) { 
    //Gets the lvgl object from the event
    lv_obj_t * dropdown = lv_event_get_target(event); 
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //Switches the auto type depending on the selectedA_index
    switch (selected_index) {
        

        case 0:
            break;

        case 1:
            //If the second option is selected sets auto type to blue
            auto_type = AUTONOMOUS_BLUE;
            break;

        case 2:
            //If the third option is selected sets auto type to red
            auto_type = AUTONOMOUS_RED;
            break;

        default:
            break;

    }

}


//Creates the action for when using the auton selector ddl
static void dropdown_auton_selector_action(lv_event_t * event) {
    //Gets the lvgl object from the event
    lv_obj_t * dropdown = lv_event_get_target(event); 
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //switches the selected program depending on the selected_index, the default is program index 3 because that is the disabled program
    switch (selected_index) {

        case 0:
            //If the first option is selected sets auton to disabled
            selected_program = 3;    
            break;

        case 1:
            //If the second option is selected sets auton to rightOneAuton of selected color
            selected_program = 0;
            break;

        case 2:
            //If the third option is selected sets auton to rightTwoAuton of selected color
            selected_program = 1;
            break;

        case 3:
            //If the fourth option is selected sets auton to rightElimAuton of selected color
            selected_program = 2;
            break;

        case 4:
            //If the fifth option is selected sets auton to disabled
            selected_program = 3;
            break;

        case 5:
            //If the sixth option is selected sets auton to leftOneAuton of selected color
            selected_program = 4;
            break;

        case 6:
            //If the seventh option is selected sets auton to leftTwoAuton of selected color
            selected_program = 5;
            break;

        case 7:
            //If the seventh option is selected sets auton to leftTwoAuton of selected color
            selected_program = 6;
            break;
        case 8:
            //If the seventh option is selected sets auton to leftTwoAuton of selected color
            selected_program = 7;
            break;
        case 9:
            //If the seventh option is selected sets auton to leftTwoAuton of selected color
            selected_program = 8;
            break;
        case 10:
            //If the seventh option is selected sets auton to leftTwoAuton of selected color
            selected_program = 9;
            break;
        default:
        //For the else statement sets auton to disabled
            selected_program = 3;
            break;

    }
}


//Creates the action for when using the skills selector ddl
static void dropdown_skill_selector_action(lv_event_t* event) {
    //Gets the lvgl object from the event
    lv_obj_t* dropdown = lv_event_get_target(event);
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //switches the auto type and selected program for skills programs
    switch(selected_index) {

        case 0:
            break;

        case 1:
            //If the second option is selected sets auto type to skills, and selected program to programming skills
            auto_type = AUTONOMOUS_SKILLS;
            selected_program = 0;
            break;

        case 2:
            //If the second option is selected sets auto type to skills, and selected program to driver skills
            auto_type = AUTONOMOUS_SKILLS;
            selected_program = 1;
            break;

        default:
            break;

    }
}


//Creates the action for when using the profile selector ddl
static void dropdown_profile_selector_action(lv_event_t* event) {
    //Gets the lvgl object from the event
    lv_obj_t* dropdown = lv_event_get_target(event);
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //switches the selected profile based of the selcted_index
    switch(selected_index) {

        case 0:
            break;

        case 1:
            //If the second option is selected, it selectes the first profile
            selected_profile = 0;
            break;

        case 2:
            //If the third option is selected, it selectes the second profile
            selected_profile = 1;
            break;

        default:
            break;

    }
}


//Creates the action for when using the debug selector ddl
static void dropdown_debug_option_selector_action (lv_event_t* event) {
    //Gets the lvgl object from the event
    lv_obj_t* dropdown = lv_event_get_target(event);
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //switches the selcted debug option based off the selected_index
    switch(selected_index){
        case 0: 
            //If the first option is selected, it sets selected_debug_option to 0
            selected_debug_option = 0;
            break;
        case 1:
            //If the second option is selected, it sets selected_debug_option to 1
            selected_debug_option = 1;
            break;
        case 2:
             /* If the third option is selected, it sets selected_debug_option to two and 
            loads the debug motor screen */
            selected_debug_option = 2;
            lv_scr_load(debug_motor_screen);
            break;
        case 3:
             /* If the fourth option is selected, it sets selected_debug_option to the and 
            loads the debug electronics screen */
            selected_debug_option = 3;
            lv_scr_load(debug_electronics_screen);
            break;
        case 4:
            // If the third option is selected, it sets selected_debug_option to four and
            selected_debug_option = 4;
            break;
        default:
            break;

    }
}


//Creates the action for when using the gif selector ddl
static void dropdown_GIF_selector_action(lv_event_t* event) {
    //Gets the lvgl object from the event
    lv_obj_t* dropdown = lv_event_get_target(event);
    // Gets the selected index of the dropdown list
    const uint16_t selected_index = lv_dropdown_get_selected(dropdown);
    //switches the selected gif based off the selected_index
    if ((selected_index != 0) && (selected_index <= gif_file_paths.size())){
        selected_GIF = selected_index-1;
    }
    else {
        selected_GIF = 5;
    }
    // printf("1\n");
    // // Create new LVGL object first
    // lv_obj_t* new_gif_obj = lv_obj_create(visual_screen);
    
    // // Setup new object
    // lv_obj_clear_flag(new_gif_obj, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_set_style_border_width(new_gif_obj, 0, LV_PART_MAIN);
    // lv_obj_set_size(new_gif_obj, 150, 113);
    // lv_obj_align(new_gif_obj, LV_ALIGN_CENTER, 150, -25);
    
    // // Style setup
    // static lv_style_t lv_style_transp;
    // lv_style_init(&lv_style_transp);
    // lv_style_set_bg_opa(&lv_style_transp, LV_OPA_TRANSP);
    // lv_obj_add_style(new_gif_obj, &lv_style_transp, 0);
    
    // // Cleanup old GIF immediately
    // gif_preview.clean();
    
    // // Create new GIF
    // gif_preview = Gif(gif_file_paths[selected_GIF], new_gif_obj);
    
    // // Delete old object after new GIF is created
    // if(gif_obj_gif) {
    //     lv_obj_del(gif_obj_gif);
    // }
    
    // // Update reference
    // gif_obj_gif = new_gif_obj;
}

//Creates the action for when a pid btnmatrix is pressed
static void btn_matrix_PID_check_pressed(lv_event_t * event) { 
    //Gets the lvgl object from the event
    lv_obj_t* btnmatrix = lv_event_get_target(event);
    //Gets which btn on the btnmatrix was selected
    const uint32_t id = lv_btnmatrix_get_selected_btn(btnmatrix);
    //Gets the text on the btn and stores teh adress of that char
    const char * TXT = lv_btnmatrix_get_btn_text(btnmatrix, id);
    //copies that pointer to a more public variable
    TXT_HOLD = TXT;
    pros::delay(10);
    //Makes the pressed variable of the btnmatrix that was typed on true
    if (btnmatrix == btnmatrix_kP) {
        pressed_kP = true;
    }

    else if (btnmatrix == btnmatrix_kI) {
        pressed_kI = true;
    }

    else {
        pressed_kD = true;
    }
}

//Method for updating the text areas of the pid btnmatrices
static void btn_matrix_PID_updater([[maybe_unused]] lv_obj_t* btnmatrix) {  

    //checks if a btn was pressed on the kP btnmatrix
    if(pressed_kP) { 
        //if it was the first press clear the text area
        if (btnmatrix_kP_first) {
            lv_textarea_set_text(kP_text_area,"");
            btnmatrix_kP_first = false;
        } 
        //else, if the back button was pressed, delete the last typed char in the ta
        else if(strcmp(TXT_HOLD, "Back") == 0) {
            lv_textarea_del_char(kP_text_area);
            
        }
        //else, if the enter button was pressed assign the text in the ta to the kP variable
        else if (strcmp(TXT_HOLD, "Enter") == 0) {
            *kP_ptr = atof(lv_textarea_get_text(kP_text_area));
        }
        // else, if a normal button was pressed, add the char on the button to the ta
        else {
            lv_textarea_add_text(kP_text_area, TXT_HOLD);
        }
        //change the variable to false to indicate the char has been type/ action has been made
        pressed_kP = false;
    }

    //else, checks if a btn was pressed on the kI btnmatrix
    else if (pressed_kI) {
        //if it was the first press clear the text area
        if (btnmatrix_kI_first) {
            lv_textarea_set_text(kI_text_area,"");
            btnmatrix_kI_first = false;
        }
        //else, if the back button was pressed, delete the last typed char in the ta
        else if(strcmp(TXT_HOLD, "Back") == 0) {
            lv_textarea_del_char(kI_text_area);
        }
        //else, if the enter button was pressed assign the text in the ta to the kP variable
        else if (strcmp(TXT_HOLD, "Enter") == 0) {
            *kI_ptr = atof(lv_textarea_get_text(kI_text_area));
            
        }
        // else, if a normal button was pressed, add the char on the button to the ta
        else {
            lv_textarea_add_text(kI_text_area,TXT_HOLD);
        }
        //change the variable to false to indicate the char has been type/ action has been made
        pressed_kI = false;
    }

    //else checks if a btn was pressed on the kD btnmatrix
    else if (pressed_kD) {
        //if it was the first press clear the text area
        if (btnmatrix_kD_first) {
            lv_textarea_set_text(kD_text_area,"");
            btnmatrix_kD_first=  false;
        }
        //else, if the back button was pressed, delete the last typed char in the ta
        else if(strcmp(TXT_HOLD, "Back") == 0) {
            lv_textarea_del_char(kD_text_area);
        }
        //else, if the enter button was pressed assign the text in the ta to the kP variable
        else if (strcmp(TXT_HOLD, "Enter") == 0) {
            *kD_ptr = atof(lv_textarea_get_text(kD_text_area));
        }
        // else, if a normal button was pressed, add the char on the button to the ta
        else {
            lv_textarea_add_text(kD_text_area,TXT_HOLD);
        }
        //change the variable to false to indicate the char has been type/ action has been made
        pressed_kD = false;
    }
}

//ACtion for when the back button on the debug motor info screen is pressed
static void motor_info_screen_back_button_action([[maybe_unused]] lv_event_t* event){
    lv_dropdown_set_selected(ddl_debug_option_selector, 0);
    lv_scr_load(debug_screen);
}

//Main Code
void initialization_display() {
    std::vector<std::string> motor_strings = {
        "Port: " + std::to_string(abs(drive::left_motors.at(0))),
        "Port: " + std::to_string(abs(drive::left_motors.at(1))),
        "Port: " + std::to_string(abs(drive::left_motors.at(2))),
        "Port: " + std::to_string(abs(drive::right_motors.at(0))),
        "Port: " + std::to_string(abs(drive::right_motors.at(1))),
        "Port: " + std::to_string(abs(drive::right_motors.at(2)))
    };

    const char* left_motor_one_text = motor_strings[0].c_str();
    const char* left_motor_two_text = motor_strings[1].c_str();
    const char* left_motor_three_text = motor_strings[2].c_str();
    const char* right_motor_one_text = motor_strings[3].c_str();
    const char* right_motor_two_text = motor_strings[4].c_str();
    const char* right_motor_three_text = motor_strings[5].c_str();

    lv_tabview_rename_tab(motor_tabview, 0, left_motor_one_text);
    lv_tabview_rename_tab(motor_tabview, 1, left_motor_two_text);
    lv_tabview_rename_tab(motor_tabview, 2, left_motor_three_text);
    lv_tabview_rename_tab(motor_tabview, 3, right_motor_one_text);
    lv_tabview_rename_tab(motor_tabview, 4, right_motor_two_text);
    lv_tabview_rename_tab(motor_tabview, 5, right_motor_three_text);
    //sets the style for a screen, with a black and gray gradiant
    static lv_style_t style_screen;
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_hex(0x000000));
    lv_style_set_bg_grad_color(&style_screen, lv_color_hex(0x3A3B3C));
    

    //gives the style to the auton screen
    lv_obj_add_style(auton_screen, &style_screen, 0);
    
    //sets the options for ddl_auton_selector
    lv_dropdown_set_options(ddl_auton_selector, " \n" "Low Goal 3+4\n" "Right 7 Ball\n" "SAWP\n" "NA\n" 
        "Disabled Red (Intake Forward)\n" "Middle Goal 3+4\n" "Left 7 Ball\n" "NA\n" "NA\n" "Disabled Red (Silver Owl)\n");                                                
    //links ddl_auton_selector to the action, dropdown_auton_selector_action to give the ddl logic.
    lv_obj_add_event_cb(ddl_auton_selector, dropdown_auton_selector_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //sets the mode of ddl_auton_selector so you can swipe through the options                                                                    
    lv_obj_set_scrollbar_mode(ddl_auton_selector, LV_SCROLLBAR_MODE_AUTO);
    //aligns ddl_auton_selector on the screen
    lv_obj_align(ddl_auton_selector, LV_ALIGN_LEFT_MID, 150, -20);

    //sets the options for ddl_color_selector
    lv_dropdown_set_options(ddl_color_selector, " \n" "Blue\n" "Red\n");
    //links ddl_color_selector to the action, dropdownColorSelectionAction to give the ddl logic
    lv_obj_add_event_cb(ddl_color_selector, dropdown_color_selection_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_color_selector on the screen
    lv_obj_align(ddl_color_selector, LV_ALIGN_LEFT_MID, 5, -20);

    //sets the options for ddl_auton
    lv_dropdown_set_options(ddl_auton, "Auton\n" "Skills\n" "Profiles\n" "Debug\n" "GIFs");
    //links ddl_auton to the action, drop_down_menuing_action to give the ddl logic
    lv_obj_add_event_cb(ddl_auton, drop_down_menuing_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //sets ddl_auton to option 1
    lv_dropdown_set_selected(ddl_auton, 0);
    //aligns ddl_auton on the screen
    lv_obj_align(ddl_auton, LV_ALIGN_TOP_LEFT, 5, 10);
    // Initialize transparent style
    static lv_style_t lv_style_transp;
    lv_style_init(&lv_style_transp);
    lv_style_set_bg_opa(&lv_style_transp, LV_OPA_TRANSP);
    // Create LVGL object for the GIF
    lv_obj_t* gifObjAuton = lv_obj_create(auton_screen);
    // Remove borders and scrollbars
    lv_obj_clear_flag(gifObjAuton, LV_OBJ_FLAG_SCROLLABLE);  // Remove scroll bars
    lv_obj_set_style_border_width(gifObjAuton, 0, LV_PART_MAIN); // Remove border

    // Set size and make transparent
    lv_obj_set_size(gifObjAuton, 150, 113);
    lv_obj_add_style(gifObjAuton, &lv_style_transp, 0);

    // Center align
    lv_obj_align(gifObjAuton, LV_ALIGN_CENTER, 150, -25);

    // Create GIF
    static const Gif gif_auton("/usd/SPSIntro.gif", gifObjAuton);

    
    //gives the style to the skills screen
    lv_obj_add_style(skills_screen, &style_screen, 0);

    //sets the options for ddl_skills
    lv_dropdown_set_options(ddl_skills, "Auton\n" "Skills\n" "Profiles\n" "Debug\n" "GIFs");
    //links ddl_skills to the action, drop_down_menuing_action to give the ddl logic
    lv_obj_add_event_cb(ddl_skills, drop_down_menuing_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_skills on the screen
    lv_obj_align(ddl_skills, LV_ALIGN_TOP_LEFT, 5, 10);

    //sets the options for ddl_skills_selector
    lv_dropdown_set_options(ddl_skills_selector, " \n" "Programming Skills\n" "Driver Skills\n");
    //links ddl_skills to the action, dropdown_skill_selector_action to give the ddl logic
    lv_obj_add_event_cb(ddl_skills_selector, dropdown_skill_selector_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_skills_selector on the screen
    lv_obj_align(ddl_skills_selector, LV_ALIGN_LEFT_MID, 5,-20);


    //gives the style to the profile screen
    lv_obj_add_style(profile_screen, &style_screen, 0);

    //sets the options for ddl_profile
    lv_dropdown_set_options(ddl_profile, "Auton\n" "Skills\n" "Profiles\n" "Debug\n" "GIFs");
    //links ddl_profile to the action, drop_down_menuing_action to give the ddl logic
    lv_obj_add_event_cb(ddl_profile, drop_down_menuing_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_profile on the screen
    lv_obj_align(ddl_profile, LV_ALIGN_TOP_LEFT, 5, 10);

    //sets the options for ddl_profile_selector
    lv_dropdown_set_options(ddl_profile_selector, " \n" "Gaston\n" "Patrick\n");
    //links ddl_profile_selector to the action, dropdown_profile_selector_action to give the ddl logic
    lv_obj_add_event_cb(ddl_profile_selector, dropdown_profile_selector_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_profile_selector on the screen
    lv_obj_align(ddl_profile_selector, LV_ALIGN_LEFT_MID, 5, -20);


    //gives the style to the debug screen
    lv_obj_add_style(debug_screen, &style_screen, 0);

    //sets the options for ddl_debug
    lv_dropdown_set_options(ddl_debug, "Auton\n" "Skills\n" "Profile\n" "Debug\n" "GIFs\n");
    //links ddl_debug to the action, drop_down_menuing_action to give the ddl logic
    lv_obj_add_event_cb(ddl_debug, drop_down_menuing_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_debug on the screen
    lv_obj_align(ddl_debug, LV_ALIGN_TOP_LEFT, 5, 10);

    //sets the options for ddl_debugSelector
    lv_dropdown_set_options(ddl_debug_option_selector, " \n" "PID Info\n" "Motor Info\n" "Electronics Info\n" 
    "Drive Train Test\n");
    //links ddl_debugSelector to the action dropdownDebugSelectorAction to give the ddl logic
    lv_obj_add_event_cb(ddl_debug_option_selector, dropdown_debug_option_selector_action, LV_EVENT_VALUE_CHANGED,  nullptr);
    //aligns ddl_debugSelector
    lv_obj_align(ddl_debug_option_selector, LV_ALIGN_TOP_LEFT, 150 ,10);

    //sets the click event of the back button
    lv_obj_add_event_cb(motor_info_screen_back_button, motor_info_screen_back_button_action, LV_EVENT_CLICKED, nullptr);
    lv_obj_align(motor_info_screen_back_button, LV_ALIGN_BOTTOM_RIGHT, -30, -10);
     //lv_obj_set_size(motor_info_screen_back_button, 150, 50);
    //Gives the button a label
    lv_obj_t* motor_info_screen_back_button_label = lv_label_create(motor_info_screen_back_button);
    lv_label_set_text(motor_info_screen_back_button_label, "Back");

    static lv_style_t style_arc;
    lv_style_init(&style_arc);
    lv_style_set_arc_rounded(&style_arc, false);
    lv_style_set_arc_width(&style_arc, 20);
    // extern const lv_img_dsc_t gradient; 
    // lv_style_set_arc_img_src(&style_arc, &gradient);  // Set the image source to your image array
    // lv_obj_add_style(drive_motor_temp_arc, &style_arc, LV_PART_INDICATOR);

    // lv_arc_set_bg_angles(drive_motor_temp_arc, 180, 360); // Set the background arc angles
    // lv_arc_set_angles(drive_motor_temp_arc, 180, 360); // Set the foreground arc angles
    // lv_arc_set_range(drive_motor_temp_arc, 0, 20);
    // lv_arc_set_value(drive_motor_temp_arc, 10);
    // lv_obj_set_style_arc_color(drive_motor_temp_arc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR); // Set the arc color
    // lv_obj_set_style_arc_width(drive_motor_temp_arc, 20, LV_PART_INDICATOR); // Set the arc width
    // lv_obj_set_style_border_width(drive_motor_temp_arc, 0, 0); // Remove the border
    // lv_obj_set_size(drive_motor_temp_arc, 100 * 2, 100 * 2); // Set the size of the arc
    // lv_obj_align(drive_motor_temp_arc, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_remove_style(drive_motor_temp_arc, NULL, LV_PART_KNOB);



    
    
    
    //gives the style to the visual screen
    lv_obj_add_style(visual_screen, &style_screen, 0);

    //sets the options for ddl_vision
    lv_dropdown_set_options(ddl_vision, "Auton\n" "Skills\n" "Profile\n" "Debug\n" "GIFs\n");
    //links ddl_vision to the action, drop_down_menuing_action to give the ddl logic
    lv_obj_add_event_cb(ddl_vision, drop_down_menuing_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //aligns ddl_vision on the screen
    lv_obj_align(ddl_vision, LV_ALIGN_TOP_LEFT, 5, 10);

    //sets the options for ddl_GIF
    lv_dropdown_set_options(ddl_GIF, " \n" "Sylvie\n" "Catapult\n" "Alliance\n" "Glitch\n" "Lightning\n" "Rumble\n" "Jelly/Blackout\n" "Pyro\n" "Casey\n" "Harold\n");
    //links ddl_GIF to the action, dropdown_GIF_selector_action to give the ddl logic
    lv_obj_add_event_cb(ddl_GIF, dropdown_GIF_selector_action, LV_EVENT_VALUE_CHANGED, nullptr);
    //sets the mode of ddl_GIF so you can swipe through the options
    lv_obj_set_scrollbar_mode(ddl_GIF, LV_SCROLLBAR_MODE_AUTO);
    //aligns ddl_GIF on the screen
    lv_obj_align(ddl_GIF, LV_ALIGN_LEFT_MID, 5, -20);
    //sets up gif preview
    // Remove borders and scrollbars
    lv_obj_clear_flag(gif_obj_gif, LV_OBJ_FLAG_SCROLLABLE);  // Remove scroll bars
    lv_obj_set_style_border_width(gif_obj_gif, 0, LV_PART_MAIN); // Remove border

    // Set size and make transparent
    lv_obj_set_size(gif_obj_gif, 150, 113);
    lv_obj_add_style(gif_obj_gif, &lv_style_transp, 0);

    // Center align
    lv_obj_align(gif_obj_gif, LV_ALIGN_CENTER, 150, -25);

    // Load the auton screen
    lv_scr_load(auton_screen);
}


//Code to run the right auton based off the checkboxes
void run_selected_auto() {
    //checks if an auton was selected
    if (selected_program == DO_NOT_RUN) {
        return;
    }
    //if so uses a switch case depending on the auto_type
    switch (auto_type) {
        case AUTONOMOUS_RED:
            printf("LEFT\n");
            //runs the function/program in the array red_scripts of the index selected_program
            red_scripts.at(selected_program)();
            break;
        case AUTONOMOUS_BLUE:
            printf("RIGHT\n");
            //runs the function/program in the array blue_scripts of the index selected_program
            blue_scripts.at(selected_program)();
            break;
        case AUTONOMOUS_SKILLS:
            printf("SKILLS\n");
            //runs the function/program in the array skills_scripts of the index selected_program
            skills_scripts.at(selected_program)();
            break;
        default:
            // Handle unexpected auto_type values
            break;
    }
}


//Creates a function to display a GIF
void run_selected_GIF() {
    //Makes the bg object transp
    static lv_style_t lv_style_transp;
    lv_style_init(&lv_style_transp);
    lv_style_set_bg_opa(&lv_style_transp, LV_OPA_TRANSP);

    //Creates a screen to display the GIF
    lv_obj_t* GIF_screen = lv_obj_create(nullptr);
    //Creates a container object on the screen to display the GIF
    lv_obj_t* gif_obj_main = lv_obj_create(GIF_screen);
    // Remove scroll bars
    lv_obj_clear_flag(gif_obj_main, LV_OBJ_FLAG_SCROLLABLE); 
    //removes border
    lv_obj_set_style_border_width(gif_obj_main, 0, LV_PART_MAIN); 
    //Sets the size of the container object
    lv_obj_set_size(gif_obj_main, 480, 250);
    //Makes the container object transparent
    lv_obj_add_style(gif_obj_main, &lv_style_transp, 0); 
    //Aligns the container object
    lv_obj_align(gif_obj_main, LV_ALIGN_TOP_LEFT, -10, -10);
    /* Creates on GIF object on the container object, the selected GIF is pulled from the array 
    gif_file_paths, and the index 
    selected_GIF, to select a GIF */
    static const Gif gif_main(gif_file_paths[selected_GIF], gif_obj_main);
   //Gif gif("/usd/mygif.gif", gif_obj_main);


    //Loads the screen
    lv_scr_load(GIF_screen);
}



//Creates a function to display the PID btnmatrices
void pid_screen_display(float& kP, float& kI, float& kD) {
    kP_ptr = &kP;
    kI_ptr = &kI;
    kD_ptr = &kD;
    //creates a style for the btnmatrices background
    static lv_style_t style_bg;
    lv_style_init(&style_bg);
    // Set background color to black
    lv_style_set_bg_color(&style_bg, lv_color_hex(0x000000)); 
    //removes horizontal and vertical padding
    lv_style_set_pad_hor(&style_bg, 0); 
    lv_style_set_pad_ver(&style_bg, 0); 

    //creates a style for the btns on the btnmatrices
    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    //Makes the btns a gradient of grey to black
    lv_style_set_bg_color(&style_btn, lv_color_make(0x30, 0x30, 0x30)); 
    lv_style_set_bg_grad_color(&style_btn, lv_color_hex(0x000000)); 
    //makes the button borders silver
    lv_style_set_border_color(&style_btn, lv_color_hex(0xC0C0C0));
    //sets the btn border width to one
    lv_style_set_border_width(&style_btn, 1); 
    //sets the btn border opacity to 50%
    lv_style_set_border_opa(&style_btn, LV_OPA_50); 

    //sets the size of the tas
    lv_obj_set_size(kP_text_area,150,50);
    lv_obj_set_size(kI_text_area,150,50);
    lv_obj_set_size(kD_text_area,150,50);
    /* takes the float values of kP, kI, and kD, the converts them to chars and store 
    those chars in their respective variables: kP_char, kI_char, and kD_char */
    sprintf(kP_char,"kP: %f",*kP_ptr);
    sprintf(kI_char,"kI: %f",*kI_ptr);
    sprintf(kD_char,"kD: %f",*kD_ptr);
    //sets the tas to be the values of the diiferent pid constants
    lv_textarea_set_text(kP_text_area,kP_char);
    lv_textarea_set_text(kI_text_area,kI_char);
    lv_textarea_set_text(kD_text_area,kD_char);
    //aligns the tas to be over the btnmatrices
    lv_obj_align(kI_text_area,LV_ALIGN_TOP_MID,0,0);
    lv_obj_align(kD_text_area,LV_ALIGN_TOP_RIGHT,0,0);

    //creates the btnmap for the btnmatrices
    static const char* num_pad[] = {
        "1", "2", "3", "\n",
        "4", "5", "6", "\n",
        "7", "8", "9", "\n",
        "Back", "0", "Enter", "\n",
        " ", ".", " ", ""
    };
    //sets the size of the btnmatrices
    lv_obj_set_size(btnmatrix_kP, 150, 150);
    lv_obj_set_size(btnmatrix_kI,150,150);
    lv_obj_set_size(btnmatrix_kD,150,150);
    //sets the btnmap to the btnmatrices
    lv_btnmatrix_set_map(btnmatrix_kP, num_pad);
    lv_btnmatrix_set_map(btnmatrix_kI, num_pad);
    lv_btnmatrix_set_map(btnmatrix_kD, num_pad);
    //sets the bg and btn styles to the btnmatrices
    lv_obj_add_style(btnmatrix_kP, &style_bg, 0);
    lv_obj_add_style(btnmatrix_kP, &style_btn, 0);
    lv_obj_add_style(btnmatrix_kI, &style_bg, 0);
    lv_obj_add_style(btnmatrix_kI, &style_btn, 0);
    lv_obj_add_style(btnmatrix_kD, &style_bg, 0);
    lv_obj_add_style(btnmatrix_kD, &style_btn, 0);
    //aligns the btnmatrices to be under the tas
    lv_obj_align(btnmatrix_kP,LV_ALIGN_TOP_LEFT,0,55);
    lv_obj_align(btnmatrix_kI,LV_ALIGN_TOP_MID,0,55);
    lv_obj_align(btnmatrix_kD,LV_ALIGN_TOP_RIGHT,0,55);

    //loads the (pid) screen
    lv_scr_load(pid_screen);
}



//Creates a method to constantlyu update the pid screen
void pid_constant_updating() {
    while (true) {
        //sets the callback method btn_matrix_PID_check_pressed to the btnmatrices to check if the button was clicked/released
        lv_obj_add_event_cb(btnmatrix_kP, btn_matrix_PID_check_pressed, LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(btnmatrix_kI, btn_matrix_PID_check_pressed, LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(btnmatrix_kD, btn_matrix_PID_check_pressed, LV_EVENT_RELEASED, nullptr);
        //added delay to remove incedental repeated presses
        pros::delay(800);
        //updates the tas based off which btns were clicked
        btn_matrix_PID_updater(btnmatrix_kP);
        btn_matrix_PID_updater(btnmatrix_kI);
        btn_matrix_PID_updater(btnmatrix_kD);
    }
}
/* Screen Clear code
lv_init();

// Create a screen object
lv_obj_t *screen = lv_scr_act();

// Clear the screen
lv_obj_clean(screen);
*/	
