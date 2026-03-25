#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "ncurses.h"
#include "unistd.h"
#include "math.h"
#include "pthread.h"

#include "tank.h"

#define ACTION_MAGNITUDE_EN
#define ML_TOP_EN
//#define CONTINOUS_FIRING

#define INITIAL_STATE_EN

#define EPISODE_LENGTH  200
#define LAYER_SIZE      2
#define REPLAY_BUF_SIZE 20000
#define BATCH_SIZE      64
#define REWARD_SIZE     3
#define STATE_SIZE      10
#define ACTION_SIZE     3
#define STEP_CONTROL    0.1f

//#define REWARD_TRAINING_NET
#define TRAIN_START_EPISODES 0
#define REWARD_CROSSOVER_EPISODES 150

#define REWARD_M_SIZE   1

#define TANK_LOWER_ID   0
#define TANK_UPPER_ID   1

tank_ctx g_tank_lower = {0};
tank_ctx g_tank_upper = {0};

void init_tg_ai(tg_ctx* ctx)
{
    int layers[LAYER_SIZE]                  = {128, 64};
#ifdef REWARD_TRAINING_NET
    int layers_reward_class[LAYER_SIZE]     = {128, 64};
#endif

    for (int i = 0; i < MAX_STATE_CNT; i++)
    {
        tg_ai_ctx* ai_ctx = &ctx->state[i].ai_ctx;

        DDPG* ddpg = ddpg_create(STATE_SIZE, 1, NULL, LAYER_SIZE, layers, LAYER_SIZE, layers, REPLAY_BUF_SIZE, BATCH_SIZE, 1);

        ai_ctx->ddpg = ddpg;
        ai_ctx->step = 0;

#ifdef REWARD_TRAINING_NET
        tg_ai_ctx* ai_ctx = &ctx->state[i].reward_classifier;

        DDPG* ddpg = ddpg_create(STATE_SIZE, REWARD_SIZE, NULL, LAYER_SIZE, layers_reward_class, LAYER_SIZE, layers_reward_class, REPLAY_BUF_SIZE, BATCH_SIZE, REWARD_M_SIZE);

        ai_ctx->ddpg = ddpg;
        ai_ctx->step = 0;
#endif
    }

    ctx->state[TANK_LOWER_ID].target_x = ((float)g_tank_lower.tg_body.max_x_boundary - (float)g_tank_lower.tg_body.width) / 2.0f;
    ctx->state[TANK_LOWER_ID].target_y = g_tank_lower.tg_body.y;

    ctx->state[TANK_UPPER_ID].target_x = ((float)g_tank_upper.tg_body.max_x_boundary - (float)g_tank_upper.tg_body.width) / 2.0f;
    ctx->state[TANK_UPPER_ID].target_y = g_tank_upper.tg_body.y;
}

void free_tg_ai(tg_ctx* ctx)
{
    for (int i = 0; i < MAX_STATE_CNT; i++)
    {
        ddpg_destroy(ctx->state[i].ai_ctx.ddpg);
#ifdef REWARD_TRAINING_NET
        ddpg_destroy(ctx->state[i].reward_classifier.ddpg);
#endif
    }
}

#define ACTION_UP   0
#define ACTION_DOWN 1
#define ACTION_IDLE 2

int get_action_index(float action)
{
    if (action > 0.33f)
        return ACTION_UP;
    else if (action < -0.33f)
        return ACTION_DOWN;

    return ACTION_IDLE;
}

void process_movement_action(float action, float* param)
{
#ifdef ACTION_MAGNITUDE_EN
    *param = action * MAX_F;
#else
    float action_scale = get_action_index(action);

    if (action_scale == ACTION_UP)
        *param = *param + STEP_CONTROL;
    else if (action_scale == ACTION_DOWN)
        *param = *param - STEP_CONTROL;

    if (*param > MAX_F)
        *param = MAX_F;
    if (*param < -MAX_F)
        *param = -MAX_F;
#endif
}

int is_action_shoot(float action)
{
#ifndef CONTINOUS_FIRING
    if (action >= 0.2f)
        return 1;
#endif
    return 0;
}

tank_ctx* get_opposite_tank(tank_ctx* tank)
{
    tank_ctx* tank_opposite = NULL;

    if (tank->id == TANK_LOWER_ID)
        tank_opposite = &g_tank_upper;
    else if (tank->id == TANK_UPPER_ID)
        tank_opposite = &g_tank_lower;

    return tank_opposite;
}

void get_reward(tg_state* state_ctx, tank_ctx* tank, float* state, float* reward, float* action)
{
#ifndef ML_TOP_EN
    if (tank->id == TANK_UPPER_ID)
    {
        for (int i = 0; i < REWARD_SIZE; i++)
            reward[i] = 0.0f;

        return;
    }
#endif

#ifdef REWARD_TRAINING_NET
    tg_ai_ctx* ai_ctx = &state_ctx->reward_classifier;

    if (ai_ctx->step == 0)
    {
        ddpg_new_episode(ai_ctx->ddpg);

        // Set episode initial state here
    }

    float* action_reward = ddpg_action(ai_ctx->ddpg, state);

    for (int i = 0; i < REWARD_SIZE; i++)
        reward[i] = action_reward[i];

#else
    tg_obj*     obj             = &tank->tg_body;
    tank_ctx*   opposite_tank   = get_opposite_tank(tank);
    tg_obj*     missle          = &opposite_tank->tg_missle;

    if (missle->on)
    {
        if (((missle->v_y > 0) && (missle->y > obj->y + obj->height)) ||
            ((missle->v_y < 0) && (missle->y + missle->height < obj->y)))
        {
            reward[0] = -fabs(obj->x - state_ctx->target_x) / state_ctx->target_x;
            reward[1] = -fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
        }
        else
        {
            float x_diff = fabs(tg_obj_dist_x_center(&tank->tg_body, missle));
            float y_diff = fabs(tg_obj_dist_y_center(&tank->tg_body, missle));

            int max_y, max_x;
            get_screen_limits(&max_x, &max_y);


            if ((missle->x + missle->width + 3 >= obj->x) &&
                (missle->x <= obj->x + obj->width + 3))
            {
                reward[0] = 2.0f*(x_diff / (float)max_x) - 1.0f;
                reward[1] = -fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
            }
            else
            {
                //float max_diag  = sqrt(max_x*max_x + max_y*max_y);
                //float dist_diag = sqrt(x_diff*x_diff + y_diff*y_diff);

                reward[0] = -fabs(obj->x - state_ctx->target_x) / state_ctx->target_x;
                reward[1] = -fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
            }
        }
    }
    else
    {
        reward[0] = -fabs(obj->x - state_ctx->target_x) / state_ctx->target_x;
        reward[1] = -fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
    }



    reward[2] = 0;

    if (action)
    {
        if (is_action_shoot(action[2]) && (tank_projectile_cool_down_ms(tank) == 0))
            reward[2] = -fabs(tg_obj_dist_x_center(&tank->tg_body, &opposite_tank->tg_body));
        else
            reward[2] = -1;
    }

    //reward[2] = ((float)tank->hits - 1.1*(float)tank->damage) / 100.0f;

/*
    if (tank->fired  > 0)
        reward[2] = ((float)tank->hits - ((float)tank->fired - (float)tank->hits)) / (float)tank->fired;

    if (tank->hits  > 0)
        reward[2] = reward[2] + ((float)tank->hits - (float)tank->damage) / 100.0f;
*/
#endif
}

void get_reward_m(tg_state* state_ctx, tank_ctx* tank, float* reward)
{
    tg_obj* obj = &tank->tg_body;

    reward[0] = 0.0f;
    reward[0] = reward[0] - fabs(obj->x - state_ctx->target_x) / state_ctx->target_x;
    reward[0] = reward[0] - fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
    reward[0] = reward[0] + ((float)tank->hits - 1.1*(float)tank->damage) / 100.0f;
    reward[0] = reward[0] / 3.0f;
}

void state_step(tg_state* state_ctx, tank_ctx* tank, float* state, float* action)
{
    tg_obj* obj = &tank->tg_body;

    process_movement_action(action[0], &obj->f_x);
    process_movement_action(action[1], &obj->f_y);

    if (is_action_shoot(action[2]))
        tank_shoot(tank);

    tg_obj_process(obj);
}

float random_target()
{
    float range = MAX_F*2.0f / STEP_CONTROL;

    int x = deepc_random_int(0, (int)range);

    return STEP_CONTROL * (float)x - 1.0f*MAX_F;
}

float feature_normalize(float val, float min, float max)
{
    // Netween -1 and 1
    return 2.0f*((val - min) / (max - min)) - 1.0f;

    // Between 0 and 1
    //return (val - min) / (max - min);
}

// 0 -> x pos
// 1 -> y pos
// 2 -> x velocity
// 3 -> y velocity
// 4 -> x force
// 5 -> y force
// 6 -> projectile cool down
void get_state(tg_state* state_ctx, tank_ctx* tank, float* state)
{
    tg_obj* obj = &tank->tg_body;

    int max_y, max_x;
    get_screen_limits(&max_x, &max_y);

    state[0] = feature_normalize(obj->x,    obj->min_x_boundary,    obj->max_x_boundary - obj->width);
    state[1] = feature_normalize(obj->y,    obj->min_y_boundary,    obj->max_y_boundary - obj->height);
    state[2] = feature_normalize(obj->v_x,  -MAX_V,                 MAX_V);
    state[3] = feature_normalize(obj->v_y,  -MAX_V,                 MAX_V);
    state[4] = feature_normalize(obj->f_x,  -MAX_F,                 MAX_F);
    state[5] = feature_normalize(obj->f_y,  -MAX_F,                 MAX_F);

    state[6] = (float)tank_projectile_cool_down_ms(tank) / (float)TANK_FIRE_COOL_DOWN_MS;

    state[7] = 0.0f;
    state[8] = 0.0f;
    state[9] = 0.0f;

    tank_ctx* opposite_tank = get_opposite_tank(tank);

    if (opposite_tank->tg_missle.on)
    {
        float x_diff = tg_obj_dist_x_center(&tank->tg_body, &opposite_tank->tg_missle);
        float y_diff = tg_obj_dist_y_center(&tank->tg_body, &opposite_tank->tg_missle);

        if (state_ctx->fire_decay < 0.0f)
            state_ctx->fire_decay = 1.0f;

        state[7] = state_ctx->fire_decay;
        state[8] = x_diff / (float)max_x;
        state[9] = y_diff / (float)max_y;

        //state_ctx->fire_decay = state_ctx->fire_decay + 0.05f;

        //if (state_ctx->fire_decay > 1.0f)
        //    state_ctx->fire_decay = 1.0f;

        state_ctx->fire_decay = state_ctx->fire_decay - 0.05f;

        if (state_ctx->fire_decay < 0.0f)
            state_ctx->fire_decay = 0.0f;
    }
    else
        state_ctx->fire_decay = -1.0f;
}

int missle_check_shoot(tank_ctx* tank)
{
    if (tank->tg_missle.on)
        return 1;

    if (deepc_random_int(0, 10) == 5)
    {
        tank_shoot(tank);
        return 1;
    }

    return 0;
}

void proccess_tank_missle(tank_ctx* tank)
{
    if (!tank->tg_missle.on)
    {
#ifdef CONTINOUS_FIRING
        if (!missle_check_shoot(tank))
#endif
            return;
    }

    if (tg_obj_process(&tank->tg_missle))
    {
        // Collision
        tank->tg_missle.on = 0;
    }

    if (tank->tg_missle.on)
    {
        tank_ctx* tank_opposite = get_opposite_tank(tank);

        if (tank_missle_collision(tank_opposite, &tank->tg_missle))
        {
            // Tank collision
            tank->tg_missle.on = 0;
            tank->hits++;
            tank_opposite->damage++;
        }
    }
}

void* ddpg_train_thread(void* arg)
{
    DDPG* ddpg = (DDPG*)arg;

    ddpg_train(ddpg, 0.99);

    ddpg_soft_update_target_networks(ddpg, .005);

    return NULL;
}

pthread_t ddpg_train_parallel(DDPG* ddpg)
{
    pthread_t th;

    pthread_create(&th, NULL, ddpg_train_thread, (void*)ddpg);

    return th;
}

void step_tg_ai_tank(tg_state* state_ctx, tank_ctx* tank)
{
    float reward[REWARD_SIZE];
    float state[STATE_SIZE];

    tg_ai_ctx* ai_ctx = &state_ctx->ai_ctx;

    if (ai_ctx->step == 0)
    {
        ddpg_new_episode(ai_ctx->ddpg);

#ifdef INITIAL_STATE_EN
        tank->tg_body.f_x = random_target();
        tank->tg_body.f_y = random_target();
#endif
    }

    get_state(state_ctx, tank, state);

    float* action = ddpg_action(ai_ctx->ddpg, state);

    state_step(state_ctx, tank, state, action);

    proccess_tank_missle(tank);

#ifdef REWARD_TRAINING_NET
    float reward_m = 0.0f;

    get_reward_m(state_ctx, tank, &reward_m);

    if (ai_ctx->episode >= REWARD_CROSSOVER_EPISODES)
    {
        get_reward(state_ctx, tank, state, reward, action);
    }
    else
    {
        for (int i = 0; i < TANK_ACTION_CNT; i++)
            reward[i] = reward_m;
    }

#else
    get_reward(state_ctx, tank, state, reward, action);
#endif

    get_state(state_ctx, tank, state);

    pthread_t th = {0};

    ai_ctx->step++;
    if (ai_ctx->step >= EPISODE_LENGTH)
    {
        ai_ctx->step = 0;
        ai_ctx->episode++;
    }

    if (ai_ctx->episode >= TRAIN_START_EPISODES)
    {
#ifdef INITIAL_STATE_EN
        if (ai_ctx->step == 0)
            ddpg_observe(ai_ctx->ddpg, action, reward, state, 1);
        else
#endif
            ddpg_observe(ai_ctx->ddpg, action, reward, state, 0);

        th = ddpg_train_parallel(ai_ctx->ddpg);
    }

#ifdef REWARD_TRAINING_NET
    pthread_t reward_th = {0};

    if (ai_ctx->episode >= REWARD_CROSSOVER_EPISODES)
    {
        // Train reward classifier
        tg_ai_ctx* rwd_class = &state_ctx->reward_classifier;

        rwd_class->step++;
        if (rwd_class->step >= EPISODE_LENGTH)
        {
            rwd_class->step = 0;
            rwd_class->episode++;
        }

        ddpg_observe(rwd_class->ddpg, reward, &reward_m, state, 0);

        reward_th = ddpg_train_parallel(rwd_class->ddpg);
    }
    else
    {
        tank->fired     = 0;
        tank->hits      = 0;
        tank->damage    = 0;
    }

    pthread_join(reward_th, NULL);
#endif

    pthread_join(th, NULL);
}

void step_tg_ai(tg_ctx* ctx)
{
    step_tg_ai_tank(&ctx->state[TANK_LOWER_ID], &g_tank_lower);
#ifdef ML_TOP_EN
    step_tg_ai_tank(&ctx->state[TANK_UPPER_ID], &g_tank_upper);
#else
    missle_check_shoot(&g_tank_upper);
    proccess_tank_missle(&g_tank_upper);
#endif
}

void tank_init_upper(tg_ctx* ctx, tank_ctx* tank)
{
    tank_obj_init(ctx, tank, COLOR_MAGENTA);

    tank->dir = TANK_DIR_DOWN;

    tank->tg_body.max_y_boundary = tank->tg_body.max_y_boundary / 2;

    tg_obj_pos_set_middle(&tank->tg_body);

    tank->tg_body.y = tank->tg_body.y - tank->tg_body.y / 2;
}

void tank_init_lower(tg_ctx* ctx, tank_ctx* tank)
{
    tank_obj_init(ctx, tank, COLOR_GREEN);

    tank->dir = TANK_DIR_UP;

    tank->tg_body.min_y_boundary = tank->tg_body.max_y_boundary / 2;

    tg_obj_pos_set_middle(&tank->tg_body);

    tank->tg_body.y = tank->tg_body.y + tank->tg_body.y / 2;
}

void start_ai_obj_test()
{
    tg_ctx ctx = {0};

    tg_init();

    tank_init_lower(&ctx, &g_tank_lower);
    tank_init_upper(&ctx, &g_tank_upper);

    g_tank_lower.tg_body.manual_process = 1;
    g_tank_upper.tg_body.manual_process = 1;

    g_tank_lower.tg_missle.manual_process = 1;
    g_tank_upper.tg_missle.manual_process = 1;

    tg_start_engine(&ctx);
}

void draw_tg_vector(int line, float* vector, int vector_size)
{
    tg_draw_text(line, "[");
    for (int i = 0; i < vector_size - 1; i++)
        tg_draw_text(line, "% .2f, ", vector[i]);
    tg_draw_text(line, "% .2f]", vector[vector_size - 1]);
}

void draw_tg_ai_status_tank(tg_state* state_ctx, tank_ctx* tank)
{
    float reward[REWARD_SIZE];
    float state[STATE_SIZE] = {0};;

    get_state(state_ctx, tank, state);

    get_reward(state_ctx, tank, state, reward, NULL);

    tg_draw_text(0, "Episode  : %d -> Fx=% .2f  Fy=% .2f", state_ctx->ai_ctx.episode, tank->tg_body.f_x, tank->tg_body.f_y);
    tg_draw_text(1, "Position : X=% .3f  Y=% .3f", tank->tg_body.x, tank->tg_body.y);

    tg_draw_text(2, "Reward   : ");
    draw_tg_vector(2, reward, REWARD_SIZE);

    tg_draw_text(3, "Wins     : %d", tank->hits);

    float reward_m[REWARD_M_SIZE];

    get_reward_m(state_ctx, tank, reward_m);

    tg_draw_text(4, "Reward M : ");
    draw_tg_vector(4, reward_m, REWARD_M_SIZE);
}

void draw_tg_ai_status(tg_ctx* ctx)
{
    tg_text_reset();

    draw_tg_ai_status_tank(&ctx->state[TANK_LOWER_ID], &g_tank_lower);

    float state[STATE_SIZE];
    get_state(&ctx->state[TANK_LOWER_ID], &g_tank_upper, state);

    tg_text_set_col(0, 40);
    tg_text_set_col(1, 40);
    tg_text_set_col(2, 40);
    tg_text_set_col(3, 40);
    tg_text_set_col(4, 40);

    draw_tg_ai_status_tank(&ctx->state[TANK_UPPER_ID], &g_tank_upper);

    tg_draw_text(5, "State U  : ");
    draw_tg_vector(5, state, STATE_SIZE);

    get_state(&ctx->state[TANK_LOWER_ID], &g_tank_lower, state);

    tg_draw_text(6, "State D  : ");
    draw_tg_vector(6, state, STATE_SIZE);
}
