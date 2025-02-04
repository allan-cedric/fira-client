// Made by Artur Coelho for Yapira UFPR on Feb 2021
// for the FIRASim simulator competition
// on the virtual IRONCUP 2021

#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"

#include "header.h"
#include "analyzer.h"
#include "math_operations.h"

// print a bot array info
void print_bot_info(bot_t bots[NUM_BOTS])
{
    for (int i = 0; i < NUM_BOTS; i++){
        printf("x: %f y: %f a: %f \n", bots[i].x, bots[i].y, bots[i].a);
        printf("vx: %f vy: %f va: %f \n", bots[i].vx, bots[i].vy, bots[i].va);
    }
    printf("\n");
}

// prints ball info
void print_ball_info(ball_t ball)
{
    printf("Ball:\n");
    printf("x: %f y: %f\n", ball.x, ball.y);
    printf("vx: %f vy: %f\n", ball.vx, ball.vy);
    printf("\n");
}

void print_field(field_t *f)
{
    printf("========================================\n");
    printf("%s\n", f->my_robots_are_yellow ? "YELLOW" : "BLUE");
    printf("%s\n\n", f->fs.wra 
            ? "WRA" : (f->fs.tra ? "TRA" : (f->fs.wrc ? "WRC" : "TRC")));
    print_ball_info(f->ball);
    printf("OUR BOTS:\n");
    print_bot_info(f->our_bots);
    printf("THEIR BOTS:\n");
    print_bot_info(f->their_bots);
}

// return the index of the biggest double in a array
int max_dist_index(double d[NUM_BOTS])
{
    int max = 0;
    for (int i = 0; i < NUM_BOTS; i++)
        if (d[max] < d[i])
            max = i;
    return max;
}

// return the index of the smallest double in a array
int min_dist_index(double d[NUM_BOTS])
{
    int min = 0;
    for (int i = 0; i < NUM_BOTS; i++)
        if (d[min] > d[i])
            min = i;
    return min;
}

// true if x is on your field, false otherwise
bool is_on_my_field(double x, bool mray)
{
    return mray ? x > MID_FIELD : x < MID_FIELD;
}

// true if the closest bot to the ball is yours, false otherwise
bool we_are_closer(field_t *f)
{
    double our_distances[NUM_BOTS];
    double their_distances[NUM_BOTS];
    float_pair_t ball_p = {.x = f->ball.x, .y = f->ball.y};

    for (int i = 0; i < NUM_BOTS; i++){
        float_pair_t bot_p = {.x = f->our_bots[i].x, .y = f->our_bots[i].y};
        our_distances[i] = vec_distance(bot_p, ball_p);
    }

    for (int i = 0; i < NUM_BOTS; i++){
        float_pair_t bot_p = {.x = f->their_bots[i].x, 
                            .y = f->their_bots[i].y };
        their_distances[i] = vec_distance(bot_p, ball_p);
    }

    f->closer_bot = &f->our_bots[min_dist_index(our_distances)];

    return our_distances[min_dist_index(our_distances)] 
            < their_distances[min_dist_index(their_distances)];
}

// true if the ball and and offender are on your field simoutaniously
// false otherwise
bool they_are_atacking(field_t *f)
{
    if (!is_on_my_field(f->ball.x, f->my_robots_are_yellow))
        return false;

    int i = 0;
    while (!is_on_my_field(f->their_bots[i].x, f->my_robots_are_yellow) 
            && (i++ < NUM_BOTS));

    return i < NUM_BOTS;
}

// true if the ball and one of your bots are on the oposite field
// false otherwise
bool we_are_atacking(field_t *f)
{
    if (is_on_my_field(f->ball.x, f->my_robots_are_yellow))
        return false;

    int i = 0;
    while (is_on_my_field(f->our_bots[i].x, f->my_robots_are_yellow) 
            && (i++ < NUM_BOTS));

    return i < NUM_BOTS;
}

// main exported function of lib
// returns the field status based on field info
void field_analyzer(field_t *f)
{
    f->fs.wrc = we_are_closer(f);
    f->fs.tra = they_are_atacking(f);
    if (!f->fs.tra){
        f->fs.wra = we_are_atacking(f);
    }
    

#ifdef ALL_META_INFO_ROBOTS
    print_field(f);
#endif

}

void referee_analyzer(RefereeClient *ref_client, referee_flags_t *ref_flags)
{
    auto get_last_foul = ref_client->getLastFoul();

    if(get_last_foul == VSSRef::Foul::HALT){
        ref_flags->is_game_on = false;
        ref_flags->is_halt = true;
    }else if(get_last_foul == VSSRef::Foul::GAME_ON){
        ref_flags->is_game_on = true;
        ref_flags->is_halt = false;
    }
    else
    {
        ref_flags->is_game_on = false;
        ref_flags->is_halt = false;
    }
}