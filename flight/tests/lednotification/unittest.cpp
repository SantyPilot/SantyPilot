#include "gtest/gtest.h"

#include <stdio.h> /* printf */
#include <stdlib.h> /* abort */
#include <string.h> /* memset */

extern "C" {
#define xTaskGetTickCount() 1
#define portTICK_RATE_MS 1

#include "lednotification.c"

void PIOS_WS2811_setColorRGB(__attribute__((unused)) Color_t c, __attribute__((unused)) uint8_t led, __attribute__((unused)) bool update) {}
void PIOS_WS2811_Update() {}
}

class LedNotificationTest : public testing::Test {};

int compare_int8(const void *a, const void *b)
{
    const int8_t *da = (const int8_t *)a;
    const int8_t *db = (const int8_t *)b;

    return (*da > *db) - (*da < *db);
}


void sort_priorities(int8_t *queued_priorities)
{
    qsort(queued_priorities, MAX_BACKGROUND_NOTIFICATIONS, sizeof(int8_t), compare_int8);
}

void set_highest_active(NotifierLedStatus_t *status)
{
    int8_t highest = NOTIFY_PRIORITY_BACKGROUND;

    for (int i = 0; i < MAX_BACKGROUND_NOTIFICATIONS; i++) {
        if (status->queued_priorities[i] > highest) {
            status->active_sequence_num = i;
            highest = status->queued_priorities[i];
        }
    }
}

void insert(NotifierLedStatus_t *status, pios_notify_priority priority)
{
    ExtLedNotification_t notification;

    notification.priority = priority;
    push_queued_sequence(&notification, status);
}

void init(NotifierLedStatus_t *status, pios_notify_priority priority)
{
    for (uint8_t i = 0; i < MAX_BACKGROUND_NOTIFICATIONS; i++) {
        status->queued_priorities[i] = priority;
    }
}

TEST_F(LedNotificationTest, TestQueueOrder1) {
    NotifierLedStatus_t status;

    init(&status, NOTIFY_PRIORITY_BACKGROUND);

    insert(&status, NOTIFY_PRIORITY_LOW);
    insert(&status, NOTIFY_PRIORITY_CRITICAL);
    insert(&status, NOTIFY_PRIORITY_LOW);
    insert(&status, NOTIFY_PRIORITY_CRITICAL);

    set_highest_active(&status);


    EXPECT_EQ(NOTIFY_PRIORITY_CRITICAL, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_CRITICAL, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_BACKGROUND, (status.queued_priorities[status.active_sequence_num]));
}

TEST_F(LedNotificationTest, TestQueueOrder2) {
    NotifierLedStatus_t status;

// Fails because insert_point and first_point will both be -1. This will also cause an array-out-of bounds at:
// 146            status->queued_priorities[insert_point] = new_notification->priority;
// 147            status->queued_sequences[insert_point]  = new_notification->sequence;
// 148            updated_sequence = insert_point;

    init(&status, NOTIFY_PRIORITY_LOW);
    status.queued_priorities[0] = NOTIFY_PRIORITY_BACKGROUND;

    insert(&status, NOTIFY_PRIORITY_REGULAR);

    set_highest_active(&status);

    EXPECT_EQ(NOTIFY_PRIORITY_REGULAR, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_BACKGROUND, (status.queued_priorities[status.active_sequence_num]));
}

TEST_F(LedNotificationTest, TestQueueOrder3) {
    NotifierLedStatus_t status;

    // Fails because queued_priorities[0] _LOW and not _REGULAR. I _thibnk_ this is a bug.
    init(&status, NOTIFY_PRIORITY_REGULAR);
    status.queued_priorities[0] = NOTIFY_PRIORITY_BACKGROUND;

    insert(&status, NOTIFY_PRIORITY_LOW);

    set_highest_active(&status);


    for (uint8_t i = 0; i < MAX_BACKGROUND_NOTIFICATIONS - 1; i++) {
        EXPECT_EQ(NOTIFY_PRIORITY_REGULAR, (status.queued_priorities[status.active_sequence_num]));
        pop_queued_sequence(&status);
    }
}

TEST_F(LedNotificationTest, TestQueueOrder4) {
    NotifierLedStatus_t status;

    init(&status, NOTIFY_PRIORITY_BACKGROUND);

    insert(&status, NOTIFY_PRIORITY_REGULAR);
    insert(&status, NOTIFY_PRIORITY_REGULAR);
    insert(&status, NOTIFY_PRIORITY_REGULAR);
    insert(&status, NOTIFY_PRIORITY_LOW);

    set_highest_active(&status);

    EXPECT_EQ(NOTIFY_PRIORITY_REGULAR, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_REGULAR, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_REGULAR, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_LOW, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_BACKGROUND, (status.queued_priorities[status.active_sequence_num]));
    pop_queued_sequence(&status);
    EXPECT_EQ(NOTIFY_PRIORITY_BACKGROUND, (status.queued_priorities[status.active_sequence_num]));
}
