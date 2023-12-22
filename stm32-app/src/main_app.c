#include "debug.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "semphr.h"
#include "param.h"
#include "log.h"
#include "queue.h"
#include "estimator_kalman.h"
#include "crtp_commander_high_level.h"

#include "tof_matrix.h"
#include "trajectory.h"
#include "exploration.h"
#include "scan.h"
#include "scan_log.h"
#include "corner_detector.h"
#include "radio_comm.h"
#include "spi_comm.h"
#include "tof_daq.h"
#include "deck.h"


// extern int16_t tof_data_buf[4][8][8];
SemaphoreHandle_t tofSemaphore0;
SemaphoreHandle_t tofSemaphore1;
uint8_t ready = 0;
uint8_t land = 0;

PARAM_GROUP_START(cmds)
                PARAM_ADD(PARAM_UINT8, ready, &ready)
                PARAM_ADD(PARAM_UINT8, land, &land)
PARAM_GROUP_STOP(cmds)

static void mission_task(void *parameters);

STATIC_MEM_TASK_ALLOC(mission_task, 2000);
STATIC_MEM_TASK_ALLOC(fly_task, 1000);
STATIC_MEM_TASK_ALLOC(tof_task, 500);

QueueHandle_t commandQueue;

typedef enum {
    EXPLORE_STATE = 0,
    NEW_SCAN_STATE = 1,
    LOOP_CLOSURE_SCAN_STATE = 2,
    SCANNING_STATE = 3,
    OPTIMIZE_STATE = 4,
    LAND_STATE = 5,
};

void update_poses_pc();

void appMain() {
    vSemaphoreCreateBinary(tofSemaphore0);
    vSemaphoreCreateBinary(tofSemaphore1);
    xSemaphoreTake(tofSemaphore0, 0);
    xSemaphoreTake(tofSemaphore1, 0);
    commandQueue = xQueueCreate(1, sizeof(uint8_t));

    // STATIC_MEM_TASK_CREATE(fly_task, fly_task, "fly", NULL, 0);
    STATIC_MEM_TASK_CREATE(mission_task, mission_task, "mission", NULL, 1);
    STATIC_MEM_TASK_CREATE(tof_task, tof_task, "tof", NULL, 3);

    while (1) vTaskDelay(10000);
}

void mission_task(void *parameters) {
    // SPI init
    pinMode(DECK_GPIO_IO1, OUTPUT);  // Init CS pin
    pinMode(DECK_GPIO_IO2, INPUT);  // Init Busy pin
    digitalWrite(DECK_GPIO_IO1, 1);
    spiBegin();

    // Wait for ready command from client
    DEBUG_PRINT("Waiting for ready command...\n");
    while (!ready) vTaskDelay(M2T(100));
    while(!tof_is_init()) vTaskDelay(M2T(10));
    DEBUG_PRINT("Start\n");

    // Start mission
    estimatorKalmanInit();
    vTaskDelay(M2T(1000));
    crtpCommanderHighLevelTakeoffWithVelocity(0.6f, 0.3f, false);

    vTaskDelay(2500);
    STATIC_MEM_TASK_CREATE(fly_task, fly_task, "fly", NULL, 2);

    // Define vars
    uint8_t state = NEW_SCAN_STATE;
    int16_t loop_cnt = 0;
    int16_t cur_pose_id;
    int16_t scan_info[2];
    TickType_t scan_start_ts;

    // Define structs
    full_pose_t full_pose = {0};
    scan_frame_t scan_frame = {0};
    icp_job_t icp_job = {0};
    slam_job_t slam_job = {0};
    scan_info_t match_scan = {.pose_id = -1, .x = 0.0f, .y = 0.0f, .yaw = 0.0f};

    // Send command for the first scan
    push_to_command_queue(STOP_SPIN_CMD);
    vTaskDelay(1000);

    TickType_t prev_time = 0;
    while (1) {
        // Get ToF data and send it to PC
        xSemaphoreTake(tofSemaphore0, portMAX_DELAY);
        if (loop_cnt++ % 2 > 0) continue;
    
        TickType_t loop_time = xTaskGetTickCount() - prev_time;
        if (loop_time > 150)
            DEBUG_PRINT("Loop time is %d\n", loop_time);
        prev_time = xTaskGetTickCount();

        get_full_pose(tof_data_buf, &full_pose);
        send_burst_data(0, &full_pose, sizeof(full_pose_t)); // to PC
        spi_send_pose(&full_pose); // to GAP9
        vTaskDelay(M2T(2));

        switch (state) {
            case EXPLORE_STATE:
                if (land) {
                    state = LAND_STATE;
                    break;
                }
                // Project the ToF ranges to get the one frame
                scan_extract_points(full_pose.tof, &scan_frame);

                 // Check for existing scan nearby
                uint8_t is_scan_nearby = check_for_nearby_scans(0.6f, &match_scan);
                if (is_scan_nearby && last_scan_further_than(1.0f)) {
                    DEBUG_PRINT("Going to (%.2f, %.2f)\n", match_scan.x, match_scan.y);
                    go_to(match_scan.x, match_scan.y);

                    // Stop and spin drone
                    push_to_command_queue(STOP_SPIN_CMD);
                    vTaskDelay(M2T(1000));
                    state = LOOP_CLOSURE_SCAN_STATE;
                }
                else if (corner_detected_hough(&scan_frame) && last_scan_further_than(1.2f)) {
                    // Stop and spin drone
                    push_to_command_queue(STOP_SPIN_CMD);
                    vTaskDelay(M2T(1000));
                    state = NEW_SCAN_STATE;
                }
                break;

            case NEW_SCAN_STATE:
                cur_pose_id = get_last_pose_id();
                add_scan_to_log(full_pose.pos[0], full_pose.pos[1], full_pose.pos[2], cur_pose_id);
                scan_info[0] = cur_pose_id;
                scan_info[1] = -1;
                DEBUG_PRINT("%d:New scan with id %d: (%.2f, %.2f)\n", xTaskGetTickCount(), 
                    cur_pose_id, full_pose.pos[0], full_pose.pos[1]);
                scan_start_ts = xTaskGetTickCount();
                state = SCANNING_STATE;
                break;

            case LOOP_CLOSURE_SCAN_STATE:
                cur_pose_id = get_last_pose_id();
                add_scan_to_log(full_pose.pos[0], full_pose.pos[1], full_pose.pos[2], cur_pose_id);
                scan_info[0] = cur_pose_id;
                scan_info[1] = match_scan.pose_id;
                DEBUG_PRINT("%d: Added scan %d, match with %d: (%.2f, %.2f)\n", xTaskGetTickCount(),
                    cur_pose_id, match_scan.pose_id, full_pose.pos[0], full_pose.pos[1]);
                scan_start_ts = xTaskGetTickCount();
                state = SCANNING_STATE;
                break;

            case SCANNING_STATE:
                if (xTaskGetTickCount() - scan_start_ts > 3000) {
                    if (scan_info[1] >= 0)
                        state = OPTIMIZE_STATE;
                    else
                        state = EXPLORE_STATE;
                }
                break;

            case OPTIMIZE_STATE:
                DEBUG_PRINT("%d: Opt starts\n", xTaskGetTickCount());
                // push_to_command_queue(STOP_CMD);
                vTaskDelay(M2T(5));

                // ICP
                spi_send_scan_info(cur_pose_id, match_scan.pose_id, &icp_job); // to GAP9
                DEBUG_PRINT("ICP:\n  Result:%.2f %.2f %.2f\n", icp_job.result[0], icp_job.result[1], icp_job.result[2]);
                DEBUG_PRINT("  Error: %.3f\n", icp_job.error);
                DEBUG_PRINT("  Time: %d\n", icp_job.time_ms);
                DEBUG_PRINT("  Has nan: %d\n\n", icp_job.has_nan);
                vTaskDelay(M2T(3));

                // SLAM
                spi_send_opt_cmd(&slam_job); 
                DEBUG_PRINT("  Pose Nr.: %d\n", slam_job.number_of_poses);
                DEBUG_PRINT("  Main Graph: %d\n", slam_job.main_graph_size);
                DEBUG_PRINT("  Subgraph: %d\n", slam_job.max_subgraph_size);
                DEBUG_PRINT("  Time: %d\n", slam_job.time_ms);
                DEBUG_PRINT("  Has nan: %d\n\n", slam_job.has_nan);
                vTaskDelay(M2T(3));

                while (is_spinning()) vTaskDelay(10);

                if (icp_job.has_nan || slam_job.has_nan || (icp_job.error >= 0.01)) {
                    state = EXPLORE_STATE;
                    break;
                }

                // Update EKF
                correct_position();

                // Update scan poses
                update_optimized_poses();
                DEBUG_PRINT("%d: Opt ends\n", xTaskGetTickCount());
                vTaskDelay(M2T(3));
                
                vTaskDelay(100);
                state = EXPLORE_STATE;
                break;

            case LAND_STATE:
                push_to_command_queue(EXIT_CMD);
                vTaskDelay(1000);
                crtpCommanderHighLevelLand(0.0f, 2.0f);
                update_poses_pc();
                while (1) vTaskDelay(1000);

                break;
        }
    }

    while (1) vTaskDelay(1000);
}


void update_poses_pc() {
    int16_t pose_count = get_last_pose_id() + 1;
    for (int16_t i=0; i<pose_count; i++) {
        float poses[3];
        spi_rcv_poses(poses, i, 1);
        vTaskDelay(M2T(2));
        uint8_t payload[14];
        memcpy(&payload[0], &i, sizeof(int16_t));
        memcpy(&payload[2], poses, 3 * sizeof(float));
        send_burst_data(3, payload, 14);
        if (i%100 == 0) DEBUG_PRINT("%d pose updates sent to pc\n", i);
    }
}