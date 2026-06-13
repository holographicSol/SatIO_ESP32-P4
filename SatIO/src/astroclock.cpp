#include "astroclock.h"
#include "lvgl.h"
#include <math.h>
#include "./sidereal_helper.h"
#include "./meteors.h"
#include "./satio.h"
#include "./wtgps300p.h"
#include "./SiderealPlanets.h"
#include "./SiderealObjects.h"
#include <cmath>
#include "esp_heap_caps.h"
#include "satio_lvgl.h"

// External data - adjust these includes to match your project
extern "C" {
    // Declare your sidereal data structures here or include the proper header
}

LV_FONT_DECLARE(cobalt_alien_17);
LV_FONT_DECLARE(cobalt_alien_25);

// ============================================================================
// CONFIGURATION
// ============================================================================

// Angle offset for positioning
int ANGLE_OFFSET = 90;

// Dimensions for the astro clock display
int32_t ASTRO_WIDTH  = 556;
int32_t ASTRO_HEIGHT = 556;

// Dimensions of the outline around the astro clock display
int32_t OUTLINE_WIDTH  = 720;
int32_t OUTLINE_HEIGHT = 720;

// Center of astro clock display
int32_t SOLAR_CENTER_X = ASTRO_WIDTH / 2;
int32_t SOLAR_CENTER_Y = ASTRO_HEIGHT / 2;

// Max usable orbit radius (leave margin for planet size)
int32_t MAX_ORBIT_RADIUS = (ASTRO_WIDTH < ASTRO_HEIGHT ? ASTRO_WIDTH : ASTRO_HEIGHT) / 2 - 15;

// Orbit step size (8 planetary orbits from Mercury to Neptune)
int32_t ORBIT_STEP = MAX_ORBIT_RADIUS / 8;

// Planet size unit (max planet radius should fit between orbits)
int32_t SIZE_UNIT = ORBIT_STEP / 6;

// Current target selection
int32_t current_target = 0;

// Timer for astro clock updates
lv_timer_t * astro_timer = NULL;

static float luna_sat_progress = 0.0f;
static int luna_sat_direction = 1;
uint16_t current_luna_hue=340;
uint16_t current_luna_saturation=20;
lv_color_t rainbow_luna_hue;

static float sun_hue_progress = 0.0f;
static int sun_sat_direction = 1;
uint32_t current_sun_hue=60;
lv_color_t rainbow_sun_hue;

// ============================================================================
// COLORS
// ============================================================================
#define COLOR_ORBIT_BELOW      lv_color_make(  0,   0,  96)
#define COLOR_ORBIT_ABOVE      lv_color_make(  0, 128,   0)
#define COLOR_SUN_BELOW        lv_color_make(128,   0,   0)
#define COLOR_SUN_ABOVE        lv_color_make(128,   0,   0)
#define COLOR_ORBIT_LUNA_BELOW lv_color_make(  0,   0,  96)
#define COLOR_ORBIT_LUNA_ABOVE lv_color_make(  0, 128,   0)
#define COLOR_TARGET           lv_color_make(255,   0,   0)
#define COLOR_ZODIAC           lv_color_make(  0,   0,  96)

#define COLOR_HAZARD           lv_color_make(255, 255,   0)
#define COLOR_WARNING          lv_color_make(255,   0,   0)

// ============================================================================
// PLANET DATA STRUCTURE
// ============================================================================
typedef struct {
    int orbit_radius;
    int radius;
    int x, y;
    lv_color_t color;
    lv_obj_t * obj;
    lv_obj_t * orbit;
    lv_obj_t * target_box;
} Planet;

#define ZODIAC_LINE_WIDTH     2
#define TARGET_BOX_LINE_WIDTH 2

#define ORBIT_ARC_WIDTH_BELOW 2
#define ORBIT_ARC_WIDTH_ABOVE 2

#define LUNA_ORBIT_ARC_WIDTH_BELOW 3
#define LUNA_ORBIT_ARC_WIDTH_ABOVE 3

#define SUN_ALTITUDE_LINE_WIDTH_BELOW 3
#define SUN_ALTITUDE_LINE_WIDTH_AVOVE 3

#define ASTRO_INFO_LINE_WIDTH 2

// ============================================================================
// PLANETS (orbits and sizes calculated from available space)
// Size scale: Sun=8, Jupiter=6, Saturn/Earth=5, Venus/Mars/Uranus/Neptune=4, Mercury=3, Luna=2
// Scaled by SIZE_UNIT to fit within orbit spacing
// ============================================================================
static Planet sun     = {0,              SIZE_UNIT * 8 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Largest
static Planet mercury = {ORBIT_STEP * 1, SIZE_UNIT * 3 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Smallest planet
static Planet venus   = {ORBIT_STEP * 2, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet earth   = {ORBIT_STEP * 3, SIZE_UNIT * 5 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet luna    = {ORBIT_STEP / 2, SIZE_UNIT * 3 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Relative to Earth
static Planet mars    = {ORBIT_STEP * 4, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet jupiter = {ORBIT_STEP * 5, SIZE_UNIT * 6 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Largest planet
static Planet saturn  = {ORBIT_STEP * 6, SIZE_UNIT * 5 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet uranus  = {ORBIT_STEP * 7, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet neptune = {ORBIT_STEP * 8, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};

typedef struct {
    int32_t size_w;
    int32_t size_h;    
    int32_t pos_x;
    int32_t pos_y;
    lv_obj_t * label;
    lv_obj_t * target_box;
} Indicator;

static Indicator meteors_indicator = {};

// ============================================================================
// LVGL OBJECTS
// ============================================================================
static lv_obj_t * volatile astro_container = NULL;
static lv_obj_t * zodiac_lines[12] = {NULL};
static lv_point_precise_t zodiac_points[12][2];

static lv_obj_t * sun_altitude_line = NULL;
lv_point_precise_t sun_altitude_points[2];

static lv_obj_t * mercury_altitude_line = NULL;
lv_point_precise_t mercury_altitude_points[2];

static lv_obj_t * venus_altitude_line = NULL;
lv_point_precise_t venus_altitude_points[2];

static lv_obj_t * earth_altitude_line = NULL;
lv_point_precise_t earth_altitude_points[2];

static lv_obj_t * luna_altitude_line = NULL;
lv_point_precise_t luna_altitude_points[2];

static lv_obj_t * mars_altitude_line = NULL;
lv_point_precise_t mars_altitude_points[2];

static lv_obj_t * jupiter_altitude_line = NULL;
lv_point_precise_t jupiter_altitude_points[2];

static lv_obj_t * saturn_altitude_line = NULL;
lv_point_precise_t saturn_altitude_points[2];

static lv_obj_t * uranus_altitude_line = NULL;
lv_point_precise_t uranus_altitude_points[2];

static lv_obj_t * neptune_altitude_line = NULL;
lv_point_precise_t neptune_altitude_points[2];

static lv_obj_t * luna_shadow = NULL;  // Shadow overlay for luna phase
static lv_obj_t * saturn_ring = NULL;  // Saturn's rings
static lv_point_precise_t saturn_ring_points[2];



// Target data display objects
static lv_obj_t * target_data_box = NULL;       // Box to display target object data
static lv_obj_t * target_connector_line = NULL; // Line connecting target box to data box
static lv_point_precise_t connector_points[2];

// Data box margin
#define DATA_BOX_MARGIN 10

// ============================================================================
// PERIMETER CLOCK
// ============================================================================

#define CLOCK_DOT_COUNT 60

static lv_obj_t * clock_dots[CLOCK_DOT_COUNT] = {NULL};
static int32_t    clock_ring_radius = 0;
static int32_t    clock_dot_r_small = 0;
static int32_t    clock_dot_r_large = 0;

#define COLOR_CLOCK_INACTIVE lv_color_make(  0,  48,   0)
#define COLOR_CLOCK_ACTIVE   lv_color_make(  0, 255,   0)

// ============================================================================
// TO RADIANS
// ============================================================================
static inline float deg2rad(float deg) {return deg * M_PI / 180.0f;}
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI;}

// ============================================================================
// CELESTIAL OBJECT CLICK CALLBACK
// ============================================================================
static void celestial_click_cb(lv_event_t * e) {
    int target = (int)(intptr_t)lv_event_get_user_data(e);
    astro_clock_set_target(target);
}

// ============================================================================
// CONTAINER CLICK CALLBACK (reset target when clicking background)
// ============================================================================
static void container_click_cb(lv_event_t * e) {
    lv_obj_t * target_obj = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t * current_obj = (lv_obj_t *)lv_event_get_current_target(e);
    // Only reset if clicking directly on container (not on a child object)
    if (target_obj == current_obj) {
        astro_clock_set_target(ASTRO_TARGET_NONE);
    }
}

// ============================================================================
// CREATE PLANET CIRCLE
// ============================================================================
static lv_obj_t * create_planet(lv_obj_t * parent, int radius, lv_color_t color) {
    printf("DEBUG: creating planet\n");
    lv_obj_t * obj = lv_obj_create(parent);
    if (!obj) {printf("ERROR: Failed to create planet\n"); return NULL;}
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, radius * 2, radius * 2);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN); // Hidden until position update
    return obj;
}

// ============================================================================
// CREATE ORBIT ARC
// ============================================================================
static lv_obj_t * create_orbit(lv_obj_t * parent, int radius, lv_color_t color) {
    printf("DEBUG: creating arc\n");
    lv_obj_t * arc = lv_arc_create(parent);
    if (!arc) {printf("ERROR: Failed to create arc\n"); return NULL;}
    lv_obj_remove_style_all(arc);
    lv_obj_set_size(arc, radius * 2, radius * 2);
    lv_obj_set_style_arc_color(arc, color, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, ORBIT_ARC_WIDTH_BELOW, LV_PART_MAIN);  // Slightly wider for smoother appearance
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_value(arc, 0);
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    return arc;
}

static Indicator create_indicator(
    lv_obj_t * parent,
    int32_t size_w,
    int32_t size_h,
    int32_t pos_x,
    int32_t pos_y,
    const char * text
    )
{
    Indicator result = {};

    result.pos_x = pos_x;
    result.pos_y = pos_y;
    result.size_w = size_w;
    result.size_h = size_h;

    result.label = lv_label_create(parent);

    // Size and position
    lv_obj_set_size(result.label, size_w, size_h);
    lv_obj_set_pos(result.label, pos_x, pos_y);

    // Main style: radius
    lv_obj_set_style_radius(result.label, 5, LV_PART_MAIN);

    // Vertical centering: calculate top padding based on font height
    int32_t font_line_height = lv_font_get_line_height(&cobalt_alien_17) * 1;
    int32_t pad_top = (size_h - font_line_height) / 2;
    if (pad_top > 0) {
        lv_obj_set_style_pad_top(result.label, pad_top, LV_PART_MAIN);
    }

    // Main style: outline
    lv_obj_set_style_outline_width(result.label, 2, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.label, COLOR_HAZARD, LV_PART_MAIN);
    
    // Main style: border
    lv_obj_set_style_border_width(result.label, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.label, COLOR_ZODIAC, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(result.label, LV_OPA_100, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.label, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.label, COLOR_ZODIAC, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.label, &cobalt_alien_17, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label, COLOR_HAZARD, LV_PART_MAIN);
    lv_label_set_text(result.label, text);

    lv_obj_add_flag(result.label, LV_OBJ_FLAG_CLICKABLE);

    return result;
}

// ============================================================================
// CREATE TARGET BOX
// ============================================================================
static lv_obj_t * create_target_box(lv_obj_t * parent, int size) {
    if (!parent || !lv_obj_is_valid(parent)) {
        printf("ERROR: create_target_box called with invalid parent (ptr=%p)\n", (void*)parent); return NULL;
    }

    printf("DEBUG: create_target_box parent=%p size=%d\n", (void*)parent, size);
    lv_obj_t * box = lv_obj_create(parent);
    if (!box) {printf("ERROR: Failed to create target box\n"); return NULL;}

    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, size + 8, size + 8);
    lv_obj_set_style_border_width(box, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_border_color(box, COLOR_TARGET, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_CLICKABLE);
    printf("DEBUG: create_target_box created box=%p\n", (void*)box);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    return box;
}

// ============================================================================
// UPDATE PLANET POSITION
// ============================================================================
static void update_planet_pos(Planet * p, float angle_deg, int cx, int cy) {
    if (!p->obj) return;
    float rad = deg2rad(angle_deg + ANGLE_OFFSET);
    p->x = cx + (int)(p->orbit_radius * sinf(rad)) - p->radius;
    p->y = cy + (int)(p->orbit_radius * cosf(rad)) - p->radius;
    lv_obj_set_pos(p->obj, p->x, p->y);
    if (p->target_box) {
        lv_obj_set_pos(p->target_box, p->x - 4, p->y - 4);
    }
}

// ============================================================================
// ZODIAC
// ============================================================================
static void create_zodiac(lv_obj_t * parent) {
    for (int i = 0; i < 12; i++) {
        zodiac_lines[i] = lv_line_create(parent);
        lv_obj_set_style_line_color(zodiac_lines[i], COLOR_ZODIAC, 0);
        lv_obj_set_style_line_width(zodiac_lines[i], ZODIAC_LINE_WIDTH, 0);
        zodiac_points[i][0].x = 0;
        zodiac_points[i][0].y = 0;
        zodiac_points[i][1].x = 0;
        zodiac_points[i][1].y = 0;
    }
}
static void hide_zodiac() {
    for (int i = 0; i < 12; i++) {
        if (zodiac_lines[i]) {
            lv_obj_add_flag(zodiac_lines[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
static void show_zodiac() {
    for (int i = 0; i < 12; i++) {
        if (zodiac_lines[i]) {
            lv_obj_clear_flag(zodiac_lines[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
static void update_zodiac(int earth_x, int earth_y) {
    const float step = 2.0f * M_PI / 12.0f;
    const float neptune_r = (float)neptune.orbit_radius;
    
    // Iterate through 12 zodiac lines
    for (int i = 0; i < 12; i++) {
        float angle = i * step;
        float dx = cosf(angle);
        float dy = sinf(angle);
        
        // Calculate intersection of lines from Earth with Neptune's orbit circle
        float ex = (float)(earth_x - SOLAR_CENTER_X);
        float ey = (float)(earth_y - SOLAR_CENTER_Y);
        
        // Quadratic coefficients: a*t^2 + b*t + c = 0
        float a = dx * dx + dy * dy;  // = 1 for unit direction
        float b = 2.0f * (ex * dx + ey * dy);
        float c = ex * ex + ey * ey - neptune_r * neptune_r;
        
        float discriminant = b * b - 4.0f * a * c;
        float r = 0.0f;
        
        if (discriminant >= 0) {
            float sqrt_disc = sqrtf(discriminant);
            float t1 = (-b + sqrt_disc) / (2.0f * a);
            float t2 = (-b - sqrt_disc) / (2.0f * a);
            // Take positive t (direction we're drawing toward)
            if (t1 > 0 && t2 > 0) {
                r = (t1 < t2) ? t1 : t2;
            } else if (t1 > 0) {
                r = t1;
            } else if (t2 > 0) {
                r = t2;
            }
        }
        // Set points for current zodiac line
        zodiac_points[i][0].x = earth_x;
        zodiac_points[i][0].y = earth_y;
        zodiac_points[i][1].x = earth_x + (int)(dx * r);
        zodiac_points[i][1].y = earth_y + (int)(dy * r);
        
        lv_line_set_points(zodiac_lines[i], zodiac_points[i], 2);
    }
}

// ============================================================================
// Altitude
// ============================================================================
/**
 * Display altitde at users earth cooridantes (approx. direction facing
 * directly outwards into 2D space, relative to the sun).
 * 
 * Should account for extreme latitues (months of daylight/night time)
 * by using sun azimuth which is calculated using current position.
 * 
 * Due to scale, shape & perspective, this line may only accurately reflect
 * altitde direction relative to the sun, rather than also accurately
 * reflecting direction relative to all other planets.
 * 
 * Orbital path lines reflect objects that are actually above the
 * horizon (altitude > 0).
 */
static void update_altitude_line(lv_obj_t * altitude_line, float altitude_angle, lv_point_precise_t * altitude_points, float intersection) {
    if (!altitude_line) return;
    
    float rad = deg2rad(altitude_angle);
    float dx = cosf(rad);
    float dy = sinf(rad);
    
    // Calculate intersection with Neptune's orbit circle (same as zodiac lines)
    // const float neptune_r = (float)neptune.orbit_radius;
    
    float ex = (float)((earth.x + earth.radius) - SOLAR_CENTER_X);
    float ey = (float)((earth.y + earth.radius) - SOLAR_CENTER_Y);
    
    // Quadratic coefficients: a*t^2 + b*t + c = 0
    float a = dx * dx + dy * dy;  // = 1 for unit direction
    float b = 2.0f * (ex * dx + ey * dy);
    float c = ex * ex + ey * ey - intersection * intersection;
    
    float discriminant = b * b - 4.0f * a * c;
    float r = 0.0f;
    
    if (discriminant >= 0) {
        float sqrt_disc = sqrtf(discriminant);
        float t1 = (-b + sqrt_disc) / (2.0f * a);
        float t2 = (-b - sqrt_disc) / (2.0f * a);
        // Take positive t (direction we're drawing toward)
        if (t1 > 0 && t2 > 0) {
            r = (t1 < t2) ? t1 : t2;
        } else if (t1 > 0) {
            r = t1;
        } else if (t2 > 0) {
            r = t2;
        }
    }
    // Set points for altitde line (from Earth's center outwards)
    altitude_points[0].x = (earth.x + earth.radius);
    altitude_points[0].y = (earth.y + earth.radius);
    altitude_points[1].x = (earth.x + earth.radius) + (int)(dx * r);
    altitude_points[1].y = (earth.y + earth.radius) + (int)(dy * r);
    
    lv_line_set_points(altitude_line, altitude_points, 2);
}

// ============================================================================
// PERIMETER CLOCK: CREATE
// Place 60 dots around the outer ring (space between Neptune orbit and edge).
// Every 5th dot is an hour marker (slightly larger).
// ============================================================================
static void create_perimeter_clock(lv_obj_t * parent) {
    // Scale dot sizes with SIZE_UNIT like other astro elements
    // Small dots (minutes/seconds): ~SIZE_UNIT, minimum 4px radius
    // Large dots (hours, every 5th): ~SIZE_UNIT*2, minimum 6px radius
    clock_dot_r_small = (SIZE_UNIT > 4) ? SIZE_UNIT     : 4;
    clock_dot_r_large = (SIZE_UNIT > 3) ? SIZE_UNIT * 2 : 6;

    // Position ring in the gap between Neptune's orbit and the container edge
    int32_t outer_r   = (OUTLINE_WIDTH < OUTLINE_HEIGHT ? OUTLINE_WIDTH : OUTLINE_HEIGHT) / 2;
    clock_ring_radius = outer_r - clock_dot_r_large - 3;

    for (int i = 0; i < CLOCK_DOT_COUNT; i++) {
        bool    is_hour = (i % 5 == 0);
        int32_t r       = is_hour ? clock_dot_r_large : clock_dot_r_small;
        int32_t diam    = r * 2;

        // Clockwise from 12 o'clock: i=0 → top, i=15 → right, etc.
        float   angle_rad = deg2rad((float)i * 6.0f);
        int32_t cx = SOLAR_CENTER_X + (int32_t)((float)clock_ring_radius * sinf(angle_rad));
        int32_t cy = SOLAR_CENTER_Y - (int32_t)((float)clock_ring_radius * cosf(angle_rad));

        lv_obj_t * dot = lv_obj_create(parent);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, diam, diam);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, COLOR_CLOCK_INACTIVE, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_pos(dot, cx - r, cy - r);

        clock_dots[i] = dot;
    }
}

// ============================================================================
// PERIMETER CLOCK: UPDATE
// Light up the dot for current hour (12h), minute, and second in green.
// ============================================================================
static void update_perimeter_clock(void) {
    if (!clock_dots[0]) return;

    uint8_t hour   = satioData.local_hour   % 12;
    uint8_t minute = satioData.local_minute % 60;
    uint8_t second = satioData.local_second % 60;

    int32_t hour_idx   = (int32_t)hour * 5;  // maps 0-11 → 0,5,10,...,55
    int32_t minute_idx = (int32_t)minute;
    int32_t second_idx = (int32_t)second;

    for (int i = 0; i < CLOCK_DOT_COUNT; i++) {
        if (!clock_dots[i]) continue;

        bool is_hour   = (i == hour_idx);
        bool is_minute = (i == minute_idx);
        bool is_second = (i == second_idx);

        if (is_hour || is_second) {
            // Solid filled — hour or second
            lv_obj_set_style_bg_color(clock_dots[i], COLOR_CLOCK_ACTIVE, 0);
            lv_obj_set_style_bg_opa(clock_dots[i], LV_OPA_COVER, 0);
            // When minute also lands on this dot, add a white border to distinguish
            if (is_minute) {
                lv_obj_set_style_border_width(clock_dots[i], 2, 0);
            } else {
                lv_obj_set_style_border_width(clock_dots[i], 0, 0);
            }
        } else if (is_minute) {
            // Minute only: outline, transparent fill
            lv_obj_set_style_bg_opa(clock_dots[i], LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_color(clock_dots[i], COLOR_CLOCK_ACTIVE, 0);
            lv_obj_set_style_border_width(clock_dots[i], 2, 0);
        } else {
            // Inactive: dim solid fill, no border
            lv_obj_set_style_bg_color(clock_dots[i], COLOR_CLOCK_INACTIVE, 0);
            lv_obj_set_style_bg_opa(clock_dots[i], LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(clock_dots[i], 0, 0);
        }
    }
}

// ============================================================================
// UPDATE ASTRO CLOCK
// ============================================================================
void astro_clock_update(void) {

    if (!astro_container) {return;}

    // Pause timer
    // lv_timer_pause(astro_timer);

    // ------------------------------------------------------------------------
    // SUN MOON BREATHING
    // ------------------------------------------------------------------------

    // --------------------------------------------
    // Advance solar shine
    // --------------------------------------------
    current_sun_hue = 47 + (13 * sun_hue_progress);
    rainbow_sun_hue = lv_color_hsv_to_rgb(current_sun_hue, 100, 100);
    sun_hue_progress += 0.01f * sun_sat_direction;
    if (sun_hue_progress >= 1.0f) {
        sun_hue_progress = 1.0f;
        sun_sat_direction = -1;
    } else if (sun_hue_progress <= 0.0f) {
        sun_hue_progress = 0.0f;
        sun_sat_direction = 1;
    }
    lv_obj_set_style_bg_color(sun.obj, rainbow_sun_hue, LV_PART_MAIN);

    // --------------------------------------------
    // Advance luna shine
    // --------------------------------------------
    current_luna_saturation = 20 + (80 * luna_sat_progress);
    rainbow_luna_hue = lv_color_hsv_to_rgb(current_luna_hue, current_luna_saturation % 100, 100);
    luna_sat_progress += 0.01f * luna_sat_direction;
    if (luna_sat_progress >= 1.0f) {
        luna_sat_progress = 1.0f;
        luna_sat_direction = -1;
    } else if (luna_sat_progress <= 0.0f) {
        luna_sat_progress = 0.0f;
        luna_sat_direction = 1;
    }
    rainbow_luna_hue = lv_color_hsv_to_rgb(current_luna_hue, current_luna_saturation % 100, 100);
    
    // -----------------------------------------------------------------
    //                                                           MERCURY
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_mercury) {
        update_planet_pos(&mercury, siderealPlanetData.mercury_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (mercury.orbit) {
            lv_color_t color = (siderealPlanetData.mercury_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(mercury.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.mercury_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(mercury.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(mercury.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mercury.obj, LV_OBJ_FLAG_HIDDEN);
    }
    else {
        lv_obj_add_flag(mercury.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mercury.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                             VENUS
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_venus) {
        update_planet_pos(&venus, siderealPlanetData.venus_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (venus.orbit) {
            lv_color_t color = (siderealPlanetData.venus_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(venus.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.venus_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(venus.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(venus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(venus.obj, LV_OBJ_FLAG_HIDDEN); 
    }
    else {
        lv_obj_add_flag(venus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(venus.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                             EARTH
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_earth) {
        float rad = deg2rad(siderealPlanetData.earth_ecliptic_long - ANGLE_OFFSET);
        earth.x = SOLAR_CENTER_X + (int)(earth.orbit_radius * sinf(rad)) - earth.radius;
        earth.y = SOLAR_CENTER_Y + (int)(earth.orbit_radius * cosf(rad)) - earth.radius;
        lv_obj_set_pos(earth.obj, earth.x, earth.y);
        if (earth.target_box) {
            lv_obj_set_pos(earth.target_box, earth.x - 4, earth.y - 4);
        }
        if (earth.orbit) {
            lv_obj_set_style_arc_color(earth.orbit, COLOR_ORBIT_LUNA_ABOVE, LV_PART_MAIN); // change to reflect pos alt
        }

        // Draw Zodiac lines from earth
        update_zodiac((earth.x + earth.radius), (earth.y + earth.radius));

        float ecliptic_long;
        float altitde_angle;

        ecliptic_long = -siderealPlanetData.earth_ecliptic_long; // points directly at the sun
        altitde_angle = ecliptic_long - siderealPlanetData.sun_az + 180.0f; // rotate by az = deg relative to local pos, sun pos
        lv_color_t color = (siderealPlanetData.sun_alt <= 0) ? COLOR_SUN_BELOW : COLOR_SUN_ABOVE;
        lv_obj_set_style_line_color(sun_altitude_line, color, 0);
        update_altitude_line(sun_altitude_line, altitde_angle, sun_altitude_points, neptune.orbit_radius);

        lv_obj_clear_flag(earth.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(earth.obj, LV_OBJ_FLAG_HIDDEN);
        show_zodiac();
    }
    else {
        hide_zodiac();
        lv_obj_add_flag(earth.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(earth.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                              LUNA
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_luna) {
        
        // Luna uses RA mapped from 0-24 hours to 0-360 degrees
        float luna_angle = (siderealPlanetData.luna_ra / 24.0f) * 360.0f;
        float luna_rad = deg2rad(luna_angle + ANGLE_OFFSET);
        
        // Position luna relative to Earth's center
        luna.x = (earth.x + earth.radius) + (int)(luna.orbit_radius * sinf(luna_rad)) - luna.radius;
        luna.y = (earth.y + earth.radius) + (int)(luna.orbit_radius * cosf(luna_rad)) - luna.radius;
        
        lv_obj_set_pos(luna.obj, luna.x, luna.y);
        if (luna.target_box) {
            lv_obj_set_pos(luna.target_box, luna.x - 4, luna.y - 4);
        }

        lv_obj_set_style_bg_color(luna.obj, rainbow_luna_hue, LV_PART_MAIN);
        
        // Update luna phase visualization
        // luna_p: 0=New, 1=WaxCres, 2=FirstQ, 3=WaxGib, 4=Full, 5=WanGib, 6=ThirdQ, 7=WanCres
        if (luna_shadow) {
            lv_obj_set_pos(luna_shadow, luna.x, luna.y);
            int phase = (int)siderealPlanetData.luna_p;
            if (phase < 0 || phase > 7) phase = 0;  // Default to new luna if invalid
            
            // Phase determines shadow coverage and side
            // Waxing (1-3): shadow on left, shrinking
            // Waning (5-7): shadow on right, growing
            switch (phase) {
                case 0:  // New Luna - full shadow
                    lv_arc_set_bg_angles(luna_shadow, 0, 360);
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 1:  // Waxing Crescent - 75% shadow left
                    lv_arc_set_bg_angles(luna_shadow, 45, 315);
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 2:  // First Quarter - 50% shadow left
                    lv_arc_set_bg_angles(luna_shadow, 90, 270);
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 3:  // Waxing Gibbous - 25% shadow left
                    lv_arc_set_bg_angles(luna_shadow, 135, 225);
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 4:  // Full Luna - no shadow
                    lv_obj_add_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 5:  // Waning Gibbous - 25% shadow right
                    lv_arc_set_bg_angles(luna_shadow, 315, 405);  // 315 to 45 (wraps)
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 6:  // Third Quarter - 50% shadow right
                    lv_arc_set_bg_angles(luna_shadow, 270, 450);  // 270 to 90 (wraps)
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 7:  // Waning Crescent - 75% shadow right
                    lv_arc_set_bg_angles(luna_shadow, 225, 495);  // 225 to 135 (wraps)
                    lv_obj_clear_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
            }
        }
        
        // Position luna orbit arc centered on Earth
        if (luna.orbit) {
            lv_obj_align_to(luna.orbit, earth.obj, LV_ALIGN_CENTER, 0, 0);
            lv_color_t color = (siderealPlanetData.luna_alt <= 0) ? COLOR_ORBIT_LUNA_BELOW : COLOR_ORBIT_LUNA_ABOVE;
            lv_obj_set_style_arc_color(luna.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.luna_alt <= 0) ? LUNA_ORBIT_ARC_WIDTH_BELOW : LUNA_ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(luna.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(luna.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(luna.obj, LV_OBJ_FLAG_HIDDEN);

    }
    
    else {
        lv_obj_add_flag(luna.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(luna.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(luna_shadow, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                              MARS
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_mars) {
        update_planet_pos(&mars, siderealPlanetData.mars_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (mars.orbit) {
            lv_color_t color = (siderealPlanetData.mars_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(mars.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.mars_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(mars.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(mars.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mars.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    else {
        lv_obj_add_flag(mars.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mars.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                           JUPITER
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_jupiter) {
        update_planet_pos(&jupiter, siderealPlanetData.jupiter_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (jupiter.orbit) {
            lv_color_t color = (siderealPlanetData.jupiter_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(jupiter.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.jupiter_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(jupiter.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(jupiter.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(jupiter.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(jupiter.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jupiter.orbit, LV_OBJ_FLAG_HIDDEN);
    }

    // -----------------------------------------------------------------
    //                                                            SATURN
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_saturn) {
        update_planet_pos(&saturn, siderealPlanetData.saturn_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (saturn.orbit) {
            lv_color_t color = (siderealPlanetData.saturn_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(saturn.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.saturn_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(saturn.orbit, width, LV_PART_MAIN);
        }
        // Update Saturn rings position
        if (saturn_ring) {
            int ring_extent = saturn.radius + 5;  // Ring extends beyond planet
            saturn_ring_points[0].x = saturn.x + saturn.radius - ring_extent;
            saturn_ring_points[0].y = saturn.y + saturn.radius;
            saturn_ring_points[1].x = saturn.x + saturn.radius + ring_extent;
            saturn_ring_points[1].y = saturn.y + saturn.radius;
            lv_line_set_points(saturn_ring, saturn_ring_points, 2);
        }
        lv_obj_clear_flag(saturn.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(saturn.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(saturn_ring, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(saturn.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(saturn.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(saturn_ring, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                            URANUS
    // -----------------------------------------------------------------
    
    if (siderealPlanetData.track_uranus) {
        update_planet_pos(&uranus, siderealPlanetData.uranus_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (uranus.orbit) {
            lv_color_t color = (siderealPlanetData.uranus_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(uranus.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.uranus_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(uranus.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(uranus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(uranus.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(uranus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(uranus.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                           NEPTUNE
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_neptune) {
        update_planet_pos(&neptune, siderealPlanetData.neptune_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (neptune.orbit) {
            lv_color_t color = (siderealPlanetData.neptune_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(neptune.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.neptune_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(neptune.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(neptune.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(neptune.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(neptune.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(neptune.obj, LV_OBJ_FLAG_HIDDEN);
    }

    // -----------------------------------------------------------------
    //                                                           METEORS
    // -----------------------------------------------------------------

    // if (meteors_indicator.label) {
    //     // Hide indicator
    //     if (!sumMeteorShowerWarning()) {
    //         lv_obj_add_flag(meteors_indicator.label, LV_OBJ_FLAG_HIDDEN);
    //     }
    //     else {
    //         // Show indicator
    //         lv_obj_remove_flag(meteors_indicator.label, LV_OBJ_FLAG_HIDDEN);
    //         // In range
    //         if (sumMeteorShowerWarning())
    //             {lv_obj_set_style_text_color(meteors_indicator.label, COLOR_HAZARD, LV_PART_MAIN);}
    //         // Peak range overrides in range
    //         if (sumMeteorShowerPeakWarning())
    //             {lv_obj_set_style_text_color(meteors_indicator.label, COLOR_WARNING, LV_PART_MAIN);}
    //     }
    // }

    // -----------------------------------------------------------------
    //                                                    PERIMETER CLOCK
    // -----------------------------------------------------------------
    update_perimeter_clock();

    // Resume timer
    // lv_timer_resume(astro_timer);
}

// ============================================================================
// UPDATE TARGET DATA BOX CONTENT
// ============================================================================
/**
 * @brief Update the content displayed in the target data box.
 * @param target The target object ID (ASTRO_TARGET_*)
 * 
 * Called when a celestial object is selected. Add your data display code here.
 * 
 * @warning will be replaced with grids for ease of tabulation.
 */
static void update_target_data_content(int target) {
    if (!target_data_box) return;
    
    lv_obj_clean(target_data_box);
    lv_obj_t * label = lv_label_create(target_data_box);
    lv_obj_set_style_text_font(label, &cobalt_alien_17, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_make(0, 255, 0), LV_PART_MAIN);
    
    char buf[2046];
    switch (target) {
        case ASTRO_TARGET_SUN:
            snprintf(buf, sizeof(buf),
                "Sun\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.sun_r,
                siderealPlanetData.sun_s,
                siderealPlanetData.sun_distance,
                siderealPlanetData.sun_ra,
                siderealPlanetData.sun_dec,
                siderealPlanetData.sun_helio_ecliptic_lat,
                siderealPlanetData.sun_helio_ecliptic_long,
                siderealPlanetData.sun_ecliptic_lat,
                siderealPlanetData.sun_ecliptic_long,
                siderealPlanetData.sun_az,
                siderealPlanetData.sun_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_MERCURY:
            snprintf(buf, sizeof(buf),
                "Mercury\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.mercury_r,
                siderealPlanetData.mercury_s,
                siderealPlanetData.mercury_distance,
                siderealPlanetData.mercury_ra,
                siderealPlanetData.mercury_dec,
                siderealPlanetData.mercury_helio_ecliptic_lat,
                siderealPlanetData.mercury_helio_ecliptic_long,
                siderealPlanetData.mercury_ecliptic_lat,
                siderealPlanetData.mercury_ecliptic_long,
                siderealPlanetData.mercury_az,
                siderealPlanetData.mercury_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_VENUS:
            snprintf(buf, sizeof(buf),
                "Venus\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.venus_r,
                siderealPlanetData.venus_s,
                siderealPlanetData.venus_distance,
                siderealPlanetData.venus_ra,
                siderealPlanetData.venus_dec,
                siderealPlanetData.venus_helio_ecliptic_lat,
                siderealPlanetData.venus_helio_ecliptic_long,
                siderealPlanetData.venus_ecliptic_lat,
                siderealPlanetData.venus_ecliptic_long,
                siderealPlanetData.venus_az,
                siderealPlanetData.venus_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_EARTH:
            snprintf(buf, sizeof(buf),
                "Earth\n\n"
                "Rise: N/A\n"
                "Set: N/A\n"
                "Distance: %.2fm (to sea level)\n"
                "Right Ascension: N/A\n"
                "Declination: N/A\n"
                "Helio Ecliptic Latitude: N/A\n"
                "Helio Ecliptic Longitude: N/A\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: N/A\n"
                "Altitude: N/A",
                atof(gnggaData.altitude),
                siderealPlanetData.earth_ecliptic_lat,
                siderealPlanetData.earth_ecliptic_long
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_LUNA:
            snprintf(buf, sizeof(buf),
                "Luna\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Phase: %s\n"
                "Luminance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.luna_r,
                siderealPlanetData.luna_s,
                siderealPlanetData.luna_p_name[(int)siderealPlanetData.luna_p],
                siderealPlanetData.luna_lum,
                siderealPlanetData.luna_ra,
                siderealPlanetData.luna_dec,
                siderealPlanetData.luna_az,
                siderealPlanetData.luna_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_MARS:
            snprintf(buf, sizeof(buf),
                "Mars\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.mars_r,
                siderealPlanetData.mars_s,
                siderealPlanetData.mars_distance,
                siderealPlanetData.mars_ra,
                siderealPlanetData.mars_dec,
                siderealPlanetData.mars_helio_ecliptic_lat,
                siderealPlanetData.mars_helio_ecliptic_long,
                siderealPlanetData.mars_ecliptic_lat,
                siderealPlanetData.mars_ecliptic_long,
                siderealPlanetData.mars_az,
                siderealPlanetData.mars_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_JUPITER:
            snprintf(buf, sizeof(buf),
                "Jupiter\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.jupiter_r,
                siderealPlanetData.jupiter_s,
                siderealPlanetData.jupiter_distance,
                siderealPlanetData.jupiter_ra,
                siderealPlanetData.jupiter_dec,
                siderealPlanetData.jupiter_helio_ecliptic_lat,
                siderealPlanetData.jupiter_helio_ecliptic_long,
                siderealPlanetData.jupiter_ecliptic_lat,
                siderealPlanetData.jupiter_ecliptic_long,
                siderealPlanetData.jupiter_az,
                siderealPlanetData.jupiter_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_SATURN:
            snprintf(buf, sizeof(buf),
                "Saturn\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.saturn_r,
                siderealPlanetData.saturn_s,
                siderealPlanetData.saturn_distance,
                siderealPlanetData.saturn_ra,
                siderealPlanetData.saturn_dec,
                siderealPlanetData.saturn_helio_ecliptic_lat,
                siderealPlanetData.saturn_helio_ecliptic_long,
                siderealPlanetData.saturn_ecliptic_lat,
                siderealPlanetData.saturn_ecliptic_long,
                siderealPlanetData.saturn_az,
                siderealPlanetData.saturn_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_URANUS:
            snprintf(buf, sizeof(buf),
                "Uranus\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.uranus_r,
                siderealPlanetData.uranus_s,
                siderealPlanetData.uranus_distance,
                siderealPlanetData.uranus_ra,
                siderealPlanetData.uranus_dec,
                siderealPlanetData.uranus_helio_ecliptic_lat,
                siderealPlanetData.uranus_helio_ecliptic_long,
                siderealPlanetData.uranus_ecliptic_lat,
                siderealPlanetData.uranus_ecliptic_long,
                siderealPlanetData.uranus_az,
                siderealPlanetData.uranus_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_NEPTUNE:
            snprintf(buf, sizeof(buf),
                "Neptune\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.neptune_r,
                siderealPlanetData.neptune_s,
                siderealPlanetData.neptune_distance,
                siderealPlanetData.neptune_ra,
                siderealPlanetData.neptune_dec,
                siderealPlanetData.neptune_helio_ecliptic_lat,
                siderealPlanetData.neptune_helio_ecliptic_long,
                siderealPlanetData.neptune_ecliptic_lat,
                siderealPlanetData.neptune_ecliptic_long,
                siderealPlanetData.neptune_az,
                siderealPlanetData.neptune_alt
            );
            lv_label_set_text(label, buf);
            break;
        case INDICATOR_METEORS:
            char range[MAX_METEOR_SHOWERS][64];
            char peak[MAX_METEOR_SHOWERS][64];
            for (int i = 0; i < MAX_METEOR_SHOWERS; i++) {
                snprintf(range[i], sizeof(range[i]), "%s/%d %s/%d",
                        // month
                        satioData.abbrev_month_names[meteor_shower_datetime[i][0][0]-1],
                        // day
                        meteor_shower_datetime[i][0][1],
                        // month
                        satioData.abbrev_month_names[meteor_shower_datetime[i][1][0]-1],
                        // day
                        meteor_shower_datetime[i][1][1]);

                snprintf(peak[i], sizeof(peak[i]), "%s/%d %s/%d",
                        // month
                        satioData.abbrev_month_names[meteor_shower_peaks[i][0][0]-1],
                        // day
                        meteor_shower_peaks[i][0][1],
                        // month
                        satioData.abbrev_month_names[meteor_shower_peaks[i][1][0]-1],
                        meteor_shower_peaks[i][0][2]);
            }
            snprintf(buf, sizeof(buf),
                "Meteor Showers\n\n");
            int len = strlen(buf);
            for (int i = 0; i < MAX_METEOR_SHOWERS; i++) {
                if (len > sizeof(buf) - 120) break;  // safety

                len += snprintf(buf + len, sizeof(buf) - len,
                    "[ %s ]\n"
                    "    [ Range: %s ] %s\n"
                    "    [ Peak:  %s ] %s\n",
                    meteor_shower_names[i],
                    range[i],
                    meteor_shower_warning_system[i][0] ? "ACTIVE" : "INACTIVE",
                    peak[i],
                    meteor_shower_warning_system[i][1] ? "ACTIVE" : "INACTIVE"
                );
            }

            lv_label_set_text(label, buf);
            break;
    }
}

// ============================================================================
// SET TARGET
// ============================================================================
void astro_clock_set_target(int target) {
    // Hide all target boxes
    if (sun.target_box) lv_obj_add_flag(sun.target_box, LV_OBJ_FLAG_HIDDEN);
    if (mercury.target_box) lv_obj_add_flag(mercury.target_box, LV_OBJ_FLAG_HIDDEN);
    if (venus.target_box) lv_obj_add_flag(venus.target_box, LV_OBJ_FLAG_HIDDEN);
    if (earth.target_box) lv_obj_add_flag(earth.target_box, LV_OBJ_FLAG_HIDDEN);
    if (luna.target_box) lv_obj_add_flag(luna.target_box, LV_OBJ_FLAG_HIDDEN);
    if (mars.target_box) lv_obj_add_flag(mars.target_box, LV_OBJ_FLAG_HIDDEN);
    if (jupiter.target_box) lv_obj_add_flag(jupiter.target_box, LV_OBJ_FLAG_HIDDEN);
    if (saturn.target_box) lv_obj_add_flag(saturn.target_box, LV_OBJ_FLAG_HIDDEN);
    if (uranus.target_box) lv_obj_add_flag(uranus.target_box, LV_OBJ_FLAG_HIDDEN);
    if (neptune.target_box) lv_obj_add_flag(neptune.target_box, LV_OBJ_FLAG_HIDDEN);
    if (meteors_indicator.target_box) lv_obj_add_flag(meteors_indicator.target_box, LV_OBJ_FLAG_HIDDEN);
    
    // Hide data box and connector line
    if (target_data_box) lv_obj_add_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    if (target_connector_line) lv_obj_add_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    
    current_target = target;
    
    // If no target selected, we're done
    if (target == ASTRO_TARGET_NONE) return;
    
    // Get target box and object center coordinates
    lv_obj_t * box = NULL;
    int obj_center_x = 0;
    int obj_center_y = 0;
    
    switch (target) {
        case ASTRO_TARGET_SUN:
            box = sun.target_box;
            obj_center_x = SOLAR_CENTER_X;
            obj_center_y = SOLAR_CENTER_Y;
            break;
        case ASTRO_TARGET_MERCURY:
            box = mercury.target_box;
            obj_center_x = mercury.x + mercury.radius;
            obj_center_y = mercury.y + mercury.radius;
            break;
        case ASTRO_TARGET_VENUS:
            box = venus.target_box;
            obj_center_x = venus.x + venus.radius;
            obj_center_y = venus.y + venus.radius;
            break;
        case ASTRO_TARGET_EARTH:
            box = earth.target_box;
            obj_center_x = earth.x + earth.radius;
            obj_center_y = earth.y + earth.radius;
            break;
        case ASTRO_TARGET_LUNA:
            box = luna.target_box;
            obj_center_x = luna.x + luna.radius;
            obj_center_y = luna.y + luna.radius;
            break;
        case ASTRO_TARGET_MARS:
            box = mars.target_box;
            obj_center_x = mars.x + mars.radius;
            obj_center_y = mars.y + mars.radius;
            break;
        case ASTRO_TARGET_JUPITER:
            box = jupiter.target_box;
            obj_center_x = jupiter.x + jupiter.radius;
            obj_center_y = jupiter.y + jupiter.radius;
            break;
        case ASTRO_TARGET_SATURN:
            box = saturn.target_box;
            obj_center_x = saturn.x + saturn.radius;
            obj_center_y = saturn.y + saturn.radius;
            break;
        case ASTRO_TARGET_URANUS:
            box = uranus.target_box;
            obj_center_x = uranus.x + uranus.radius;
            obj_center_y = uranus.y + uranus.radius;
            break;
        case ASTRO_TARGET_NEPTUNE:
            box = neptune.target_box;
            obj_center_x = neptune.x + neptune.radius;
            obj_center_y = neptune.y + neptune.radius;
            break;
        case INDICATOR_METEORS:
            box = meteors_indicator.target_box;
            obj_center_x = meteors_indicator.pos_x + (meteors_indicator.size_w/2);
            obj_center_y = meteors_indicator.pos_y + (meteors_indicator.size_w/2);
            break;
    }
    
    // Show target box
    if (box) lv_obj_clear_flag(box, LV_OBJ_FLAG_HIDDEN);
    
    // Update data box content FIRST so we can measure its size
    update_target_data_content(target);
    lv_obj_update_layout(target_data_box);  // Force layout update to calculate size
    
    // Get actual data box dimensions after content is set
    int data_box_width = lv_obj_get_width(target_data_box);
    int data_box_height = lv_obj_get_height(target_data_box);
    
    // -----------------------------------------------------------------
    // Position data box based on object location in container
    // -----------------------------------------------------------------
    // Horizontal: Left side → data box on RIGHT, Right side → LEFT
    // Vertical: Top half → data box BELOW, Bottom half → ABOVE
    // -----------------------------------------------------------------
    
    int data_box_x, data_box_y;
    int connector_start_x, connector_start_y;
    int connector_end_x, connector_end_y;
    
    // Determine if object is on left or right side of container
    bool on_right_side = (obj_center_x > SOLAR_CENTER_X);
    // Determine if object is in top or bottom half of container
    bool in_top_half = (obj_center_y < SOLAR_CENTER_Y);
    
    // Horizontal positioning (left/right)
    if (on_right_side) {
        // Object is on right side: place data box on LEFT of object
        data_box_x = obj_center_x - data_box_width - DATA_BOX_MARGIN - 20;
        connector_start_x = obj_center_x - DATA_BOX_MARGIN;
        connector_end_x = data_box_x + data_box_width;
    } else {
        // Object is on left side or center: place data box on RIGHT of object
        data_box_x = obj_center_x + DATA_BOX_MARGIN + 20;
        connector_start_x = obj_center_x + DATA_BOX_MARGIN;
        connector_end_x = data_box_x;
    }
    
    // Vertical positioning (above/below)
    if (in_top_half) {
        // Object is in top half: place data box BELOW object
        data_box_y = obj_center_y + DATA_BOX_MARGIN + 20;
        connector_start_y = obj_center_y + DATA_BOX_MARGIN;
        connector_end_y = data_box_y;
    } else {
        // Object is in bottom half: place data box ABOVE object
        data_box_y = obj_center_y - data_box_height - DATA_BOX_MARGIN - 20;
        connector_start_y = obj_center_y - DATA_BOX_MARGIN;
        connector_end_y = data_box_y + data_box_height;
    }
    
    // Clamp data box X to container bounds
    if (data_box_x < DATA_BOX_MARGIN) data_box_x = DATA_BOX_MARGIN;
    if (data_box_x + data_box_width > ASTRO_WIDTH - DATA_BOX_MARGIN) {
        data_box_x = ASTRO_WIDTH - data_box_width - DATA_BOX_MARGIN;
    }
    
    // Clamp data box Y to container bounds
    if (data_box_y < DATA_BOX_MARGIN) data_box_y = DATA_BOX_MARGIN;
    if (data_box_y + data_box_height > ASTRO_HEIGHT - DATA_BOX_MARGIN) {
        data_box_y = ASTRO_HEIGHT - data_box_height - DATA_BOX_MARGIN;
    }
    
    // Position and show data box
    if (target_data_box) {
        lv_obj_set_pos(target_data_box, data_box_x, data_box_y);
        lv_obj_clear_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    }
    
    // Position and show connector line
    if (target_connector_line) {
        connector_points[0].x = connector_start_x;
        connector_points[0].y = connector_start_y;
        connector_points[1].x = connector_end_x;
        connector_points[1].y = connector_end_y;
        lv_line_set_points(target_connector_line, connector_points, 2);
        lv_obj_clear_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    }
}

/** ---------------------------------------------------------------------------------------
 * @brief Astro clock animation callback to update positions.
 */
static void astro_timer_cb(lv_timer_t * timer) {
    astro_clock_update();
}

// ============================================================================
// INIT ASTRO CLOCK
// ============================================================================
void astro_clock_begin(
    lv_obj_t * parent,
    int32_t outline_w_px,
    int32_t outline_h_px,
    int32_t astro_w_px,
    int32_t astro_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int angle_offset
    ) 
{
    // Guard: ensure parent is valid
    if (!parent || !lv_obj_is_valid(parent)) {
        printf("ERROR: astro_clock_begin called with invalid parent\n");
        return;
    }
    
    astro_clock_end();
    
    // Angle offset for positioning
    ANGLE_OFFSET = angle_offset;

    // Dimensions for the astro clock display
    ASTRO_WIDTH = astro_w_px;
    ASTRO_HEIGHT = astro_h_px;

    // Dimensions of the outline around the astro clock display
    OUTLINE_WIDTH = outline_w_px;
    OUTLINE_HEIGHT = outline_h_px;

    // Center of astro clock display
    SOLAR_CENTER_X = OUTLINE_WIDTH / 2;
    SOLAR_CENTER_Y = OUTLINE_HEIGHT / 2;

    // Max usable orbit radius (leave margin for planet size)
    MAX_ORBIT_RADIUS = (ASTRO_WIDTH < ASTRO_HEIGHT ? ASTRO_WIDTH : ASTRO_HEIGHT) / 2 - 15;

    // Orbit step size (8 planetary orbits from Mercury to Neptune)
    ORBIT_STEP = MAX_ORBIT_RADIUS / 8;

    // Planet size unit (max planet radius should fit between orbits)
    SIZE_UNIT = ORBIT_STEP / 6;

    // Current target selection
    current_target = 0;
    
    // Recalculate planet orbit radii and sizes (static initializers used old values)
    sun.orbit_radius = 0;
    sun.radius = SIZE_UNIT * 8 / 2;
    mercury.orbit_radius = ORBIT_STEP * 1;
    mercury.radius = SIZE_UNIT * 3 / 2;
    venus.orbit_radius = ORBIT_STEP * 2;
    venus.radius = SIZE_UNIT * 4 / 2;
    earth.orbit_radius = ORBIT_STEP * 3;
    earth.radius = SIZE_UNIT * 5 / 2;
    luna.orbit_radius = ORBIT_STEP / 2;
    luna.radius = SIZE_UNIT * 3 / 2;
    mars.orbit_radius = ORBIT_STEP * 4;
    mars.radius = SIZE_UNIT * 4 / 2;
    jupiter.orbit_radius = ORBIT_STEP * 5;
    jupiter.radius = SIZE_UNIT * 6 / 2;
    saturn.orbit_radius = ORBIT_STEP * 6;
    saturn.radius = SIZE_UNIT * 5 / 2;
    uranus.orbit_radius = ORBIT_STEP * 7;
    uranus.radius = SIZE_UNIT * 4 / 2;
    neptune.orbit_radius = ORBIT_STEP * 8;
    neptune.radius = SIZE_UNIT * 4 / 2;

    // Set colors
    sun.color = lv_color_make(255, 255, 0);
    mercury.color = lv_color_make(255, 0, 255);
    venus.color = lv_color_make(180, 180, 0);
    earth.color = lv_color_make(0, 0, 255);
    luna.color = lv_color_make(128, 128, 128);
    mars.color = lv_color_make(255, 0, 0);
    jupiter.color = lv_color_make(128, 128, 128);
    saturn.color = lv_color_make(210, 210, 0);
    uranus.color = lv_color_make(0, 255, 255);
    neptune.color = lv_color_make(255, 0, 255);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Container
    printf("DEBUG: Creating astro_container\n");
    astro_container = lv_obj_create(parent);
    if (!astro_container) { printf("ERROR: Failed to create astro_container\n"); return;}
    printf("DEBUG: astro_container done\n");
    // Target boxes - debug print before potentially crashing
    // size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    // printf("DEBUG: Creating target boxes. astro_container=%p sun.radius=%d free_heap=%u\n", 
    //        (void*)astro_container, sun.radius, free_heap);
    // // Validate astro_container before using it
    // if (!astro_container || !lv_obj_is_valid(astro_container)) {
    //     printf("ERROR: astro_container invalid before creating target boxes!\n");
    //     return;
    // }
    lv_obj_remove_style_all(astro_container);
    lv_obj_set_size(astro_container, OUTLINE_WIDTH, OUTLINE_HEIGHT);
    lv_obj_align(astro_container, alignment, pos_x, pos_y);
    lv_obj_set_style_bg_color(astro_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(astro_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(astro_container, 0, 0);
    lv_obj_set_style_border_color(astro_container, COLOR_ZODIAC, 0);
    lv_obj_set_style_outline_width(astro_container, 2, 0); // dev outline
    // lv_obj_set_style_outline_width(astro_container, 0, 0); // actual outline
    lv_obj_set_style_outline_color(astro_container, COLOR_ZODIAC, 0);
    lv_obj_remove_flag(astro_container, LV_OBJ_FLAG_SCROLLABLE);

    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Meteors Indicator
    meteors_indicator = create_indicator(
        astro_container,
        90,
        24,
        SOLAR_CENTER_X+(ORBIT_STEP * 4),
        SOLAR_CENTER_Y-(ORBIT_STEP * 9)+10,
        "METEORS"
    );
    meteors_indicator.target_box = create_target_box(astro_container, meteors_indicator.size_h*2);
    lv_obj_add_flag(meteors_indicator.label, LV_OBJ_FLAG_HIDDEN);

    // Zodiac lines
    create_zodiac(astro_container);

    // Perimeter clock dots
    create_perimeter_clock(astro_container);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Orbits (outer to inner)
    printf("DEBUG: Creating neptune.orbit\n");
    neptune.orbit = create_orbit(astro_container, neptune.orbit_radius, COLOR_ORBIT_BELOW);
    if (!neptune.orbit) { printf("ERROR: Failed to create neptune.orbit\n"); return;}
    printf("DEBUG: neptune.orbit done\n");

    printf("DEBUG: Creating uranus.orbit\n");
    uranus.orbit = create_orbit(astro_container, uranus.orbit_radius, COLOR_ORBIT_BELOW);
    if (!uranus.orbit) { printf("ERROR: Failed to create uranus.orbit\n"); return;}
    printf("DEBUG: uranus.orbit done\n");

    printf("DEBUG: Creating saturn.orbit\n");
    saturn.orbit = create_orbit(astro_container, saturn.orbit_radius, COLOR_ORBIT_BELOW);
    if (!saturn.orbit) { printf("ERROR: Failed to create saturn.orbit\n"); return;}
    printf("DEBUG: saturn.orbit done\n");

    printf("DEBUG: Creating jupiter.orbit\n");
    jupiter.orbit = create_orbit(astro_container, jupiter.orbit_radius, COLOR_ORBIT_BELOW);
    if (!jupiter.orbit) { printf("ERROR: Failed to create jupiter.orbit\n"); return;}
    printf("DEBUG: jupiter.orbit done\n");

    printf("DEBUG: Creating mars.orbit\n");
    mars.orbit = create_orbit(astro_container, mars.orbit_radius, COLOR_ORBIT_BELOW);
    if (!mars.orbit) { printf("ERROR: Failed to create mars.orbit\n"); return;}
    printf("DEBUG: mars.orbit done\n");

    printf("DEBUG: Creating earth.orbit\n");
    earth.orbit = create_orbit(astro_container, earth.orbit_radius, COLOR_ORBIT_LUNA_ABOVE);
    if (!earth.orbit) { printf("ERROR: Failed to create earth.orbit\n"); return;}
    printf("DEBUG: earth.orbit done\n");

    printf("DEBUG: Creating venus.orbit\n");
    venus.orbit = create_orbit(astro_container, venus.orbit_radius, COLOR_ORBIT_BELOW);
    if (!venus.orbit) { printf("ERROR: Failed to create venus.orbit\n"); return;}
    printf("DEBUG: venus.orbit done\n");

    printf("DEBUG: Creating mercury.orbit\n");
    mercury.orbit = create_orbit(astro_container, mercury.orbit_radius, COLOR_ORBIT_BELOW);
    if (!mercury.orbit) { printf("ERROR: Failed to create mercury.orbit\n"); return;}
    printf("DEBUG: mercury.orbit done\n");

    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Sun altitde line (line from Earth to edge showing local altitde relative to sun)
    printf("DEBUG: Creating sun_altitude_line\n");
    sun_altitude_line = lv_line_create(astro_container);
    if (!sun_altitude_line) { printf("ERROR: Failed to create sun_altitude_line\n"); return;}
    printf("DEBUG: sun_altitude_line done\n");
    lv_obj_set_style_line_color(sun_altitude_line, lv_color_make(128, 0, 0), 0);
    lv_obj_set_style_line_width(sun_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    lv_obj_set_style_line_rounded(sun_altitude_line, true, 0);

    sun_altitude_points[0].x = 0;
    sun_altitude_points[0].y = 0;
    sun_altitude_points[1].x = 0;
    sun_altitude_points[1].y = 0;

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    int luna_spacing = SIZE_UNIT;  // Gap between Earth surface and Luna orbit
    luna.orbit_radius = earth.radius + luna_spacing + luna.radius;
    luna.orbit = create_orbit(astro_container, luna.orbit_radius, COLOR_ORBIT_LUNA_BELOW);
    lv_obj_add_flag(luna.orbit, LV_OBJ_FLAG_HIDDEN);  // Hide until first update positions it

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Sun
    sun.obj = create_planet(astro_container, sun.radius, sun.color);
    lv_obj_set_pos(sun.obj, SOLAR_CENTER_X - sun.radius, SOLAR_CENTER_Y - sun.radius);
    
    // Planets
    neptune.obj = create_planet(astro_container, neptune.radius, neptune.color);
    uranus.obj = create_planet(astro_container, uranus.radius, uranus.color);
    saturn.obj = create_planet(astro_container, saturn.radius, saturn.color);
    saturn_ring = lv_line_create(astro_container);
    lv_obj_set_style_line_color(saturn_ring, lv_color_hex(0xC0A060), LV_PART_MAIN);  // Tan color
    lv_obj_set_style_line_width(saturn_ring, 2, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(saturn_ring, true, LV_PART_MAIN);
    jupiter.obj = create_planet(astro_container, jupiter.radius, jupiter.color);
    mars.obj = create_planet(astro_container, mars.radius, mars.color);
    earth.obj = create_planet(astro_container, earth.radius, earth.color);
    venus.obj = create_planet(astro_container, venus.radius, venus.color);
    mercury.obj = create_planet(astro_container, mercury.radius, mercury.color);
    luna.obj = create_planet(astro_container, luna.radius, luna.color);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Luna shadow arc for phase visualization
    luna_shadow = lv_arc_create(astro_container);
    lv_obj_remove_style_all(luna_shadow);
    lv_obj_set_size(luna_shadow, luna.radius * 2, luna.radius * 2);
    lv_obj_set_style_arc_color(luna_shadow, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_arc_width(luna_shadow, luna.radius, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(luna_shadow, false, LV_PART_MAIN);
    lv_arc_set_bg_angles(luna_shadow, 0, 180);
    lv_arc_set_value(luna_shadow, 0);
    lv_obj_set_style_arc_opa(luna_shadow, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(luna_shadow, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(luna_shadow, LV_OBJ_FLAG_CLICKABLE);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    printf("DEBUG: Creating sun.target_box\n");
    sun.target_box = create_target_box(astro_container, sun.radius * 2);
    if (!sun.target_box) { printf("ERROR: Failed to create sun.target_box\n"); return;}
    printf("DEBUG: sun.target_box done\n");
    
    printf("DEBUG: Creating mercury.target_box\n");
    mercury.target_box = create_target_box(astro_container, mercury.radius * 2);
    if (!mercury.target_box) { printf("ERROR: Failed to create mercury.target_box\n"); return;}
    printf("DEBUG: mercury.target_box done\n");
    
    printf("DEBUG: Creating venus.target_box\n");
    venus.target_box = create_target_box(astro_container, venus.radius * 2);
    if (!venus.target_box) { printf("ERROR: Failed to create venus.target_box\n"); return;}
    printf("DEBUG: venus.target_box done\n");
    
    printf("DEBUG: Creating earth.target_box\n");
    earth.target_box = create_target_box(astro_container, earth.radius * 2);
    if (!earth.target_box) { printf("ERROR: Failed to create earth.target_box\n"); return;}
    printf("DEBUG: earth.target_box done\n");
    
    printf("DEBUG: Creating luna.target_box\n");
    luna.target_box = create_target_box(astro_container, luna.radius * 2);
    if (!luna.target_box) { printf("ERROR: Failed to create luna.target_box\n"); return;}
    printf("DEBUG: luna.target_box done\n");
    
    printf("DEBUG: Creating mars.target_box\n");
    mars.target_box = create_target_box(astro_container, mars.radius * 2);
    if (!mars.target_box) { printf("ERROR: Failed to create mars.target_box\n"); return;}
    printf("DEBUG: mars.target_box done\n");
    
    printf("DEBUG: Creating jupiter.target_box\n");
    jupiter.target_box = create_target_box(astro_container, jupiter.radius * 2);
    if (!jupiter.target_box) { printf("ERROR: Failed to create jupiter.target_box\n"); return;}
    printf("DEBUG: jupiter.target_box done\n");
    
    printf("DEBUG: Creating saturn.target_box\n");
    saturn.target_box = create_target_box(astro_container, saturn.radius * 2);
    if (!saturn.target_box) { printf("ERROR: Failed to create saturn.target_box\n"); return;}
    printf("DEBUG: saturn.target_box done\n");
    
    printf("DEBUG: Creating uranus.target_box\n");
    uranus.target_box = create_target_box(astro_container, uranus.radius * 2);
    if (!uranus.target_box) { printf("ERROR: Failed to create uranus.target_box\n"); return;}
    printf("DEBUG: uranus.target_box done\n");
    
    printf("DEBUG: Creating neptune.target_box\n");
    neptune.target_box = create_target_box(astro_container, neptune.radius * 2);
    if (!neptune.target_box) { printf("ERROR: Failed to create neptune.target_box\n"); return;}
    printf("DEBUG: neptune.target_box done\n");

    // -----------------------------------------------------------------
    // Target data box (displays object information when selected)
    // -----------------------------------------------------------------
    target_data_box = lv_obj_create(astro_container);
    lv_obj_remove_style_all(target_data_box);
    lv_obj_set_size(target_data_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_border_width(target_data_box, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_border_color(target_data_box, COLOR_TARGET, 0);
    lv_obj_set_style_bg_color(target_data_box, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(target_data_box, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(target_data_box, 4, 0);
    lv_obj_add_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(target_data_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(target_data_box, LV_OBJ_FLAG_CLICKABLE);
    
    // -----------------------------------------------------------------
    // Connector line (connects target box to data box)
    // -----------------------------------------------------------------
    target_connector_line = lv_line_create(astro_container);
    lv_obj_set_style_line_color(target_connector_line, COLOR_TARGET, 0);
    lv_obj_set_style_line_width(target_connector_line, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_line_rounded(target_connector_line, true, 0);
    lv_obj_add_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    connector_points[0].x = 0;
    connector_points[0].y = 0;
    connector_points[1].x = 0;
    connector_points[1].y = 0;

    // Show sun immediately (other objects are shown after positions are set)
    lv_obj_clear_flag(sun.obj, LV_OBJ_FLAG_HIDDEN);

    // Add click event handlers to celestial objects
    lv_obj_add_event_cb(sun.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_SUN);
    lv_obj_add_event_cb(mercury.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_MERCURY);
    lv_obj_add_event_cb(venus.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_VENUS);
    lv_obj_add_event_cb(earth.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_EARTH);
    lv_obj_add_event_cb(luna.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_LUNA);
    lv_obj_add_event_cb(mars.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_MARS);
    lv_obj_add_event_cb(jupiter.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_JUPITER);
    lv_obj_add_event_cb(saturn.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_SATURN);
    lv_obj_add_event_cb(uranus.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_URANUS);
    lv_obj_add_event_cb(neptune.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_NEPTUNE);

    lv_obj_add_event_cb(meteors_indicator.label, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)INDICATOR_METEORS);

    // Add click handler to container to reset target when clicking background
    lv_obj_add_flag(astro_container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(astro_container, container_click_cb, LV_EVENT_CLICKED, NULL);

    // Create timer for astro clock updates
    astro_timer = lv_timer_create(astro_timer_cb, 500, NULL); // use delay time in config.h
}

void astro_clock_end() {

    // // Stop timer first
    // if (astro_timer) {
    //     lv_timer_pause(astro_timer);
    //     lv_timer_del(astro_timer);
    //     astro_timer = NULL;
    // }
    
    // // Delete container (this deletes all children automatically)
    // if (astro_container && lv_obj_is_valid(astro_container)) {
    //     lv_obj_del(astro_container);
    //     astro_container = NULL;
    // }

    if (astro_timer) {
        lv_timer_pause(astro_timer);
        lv_timer_delete(astro_timer);
        astro_timer = NULL;
    }
    
    // // Reset all object pointers to prevent dangling references
    // // Planet objects
    // sun.obj = NULL; sun.orbit = NULL; sun.target_box = NULL;
    // mercury.obj = NULL; mercury.orbit = NULL; mercury.target_box = NULL;
    // venus.obj = NULL; venus.orbit = NULL; venus.target_box = NULL;
    // earth.obj = NULL; earth.orbit = NULL; earth.target_box = NULL;
    // luna.obj = NULL; luna.orbit = NULL; luna.target_box = NULL;
    // mars.obj = NULL; mars.orbit = NULL; mars.target_box = NULL;
    // jupiter.obj = NULL; jupiter.orbit = NULL; jupiter.target_box = NULL;
    // saturn.obj = NULL; saturn.orbit = NULL; saturn.target_box = NULL;
    // uranus.obj = NULL; uranus.orbit = NULL; uranus.target_box = NULL;
    // neptune.obj = NULL; neptune.orbit = NULL; neptune.target_box = NULL;
    
    // // Other global objects
    // altitde_line = NULL;
    // luna_shadow = NULL;
    // saturn_ring = NULL;
    // target_data_box = NULL;
    // target_connector_line = NULL;
    
    // // Zodiac lines
    // for (int i = 0; i < 12; i++) {
    //     zodiac_lines[i] = NULL;
    // }
    
    // Reset target selection
    current_target = 0;
}