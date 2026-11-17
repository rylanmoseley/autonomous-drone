#include <gtest/gtest.h>
#include "mission_sequencer.h"

TEST(MissionSequencerTest, SetupAndAdvance) {
    MissionSequencer sequencer;
    EXPECT_TRUE(sequencer.isSequenceComplete());

    UICommandPacket pkt{};
    pkt.sequence_length = 3;
    pkt.goal_sequence[0] = SHAPE_CIRCLE;
    pkt.goal_sequence[1] = SHAPE_TRIANGLE;
    pkt.goal_sequence[2] = SHAPE_SQUARE;
    pkt.end_action = MISSION_END_LAND;

    Pose start_pose{0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    sequencer.startMission(pkt, start_pose);

    EXPECT_FALSE(sequencer.isSequenceComplete());
    EXPECT_EQ(sequencer.active_sequence_len, 3u);
    EXPECT_EQ(sequencer.current_seq_index, 0u);
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_CIRCLE);
    EXPECT_EQ(sequencer.configured_end_action, MISSION_END_LAND);
    EXPECT_TRUE(sequencer.new_goal);

    // Complete gate 0
    Pose completed_0{2.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    sequencer.advanceSequence(completed_0);
    EXPECT_EQ(sequencer.current_seq_index, 1u);
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_TRIANGLE);

    // Complete gate 1
    Pose completed_1{4.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    sequencer.advanceSequence(completed_1);
    EXPECT_EQ(sequencer.current_seq_index, 2u);
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_SQUARE);

    // Complete gate 2
    Pose completed_2{6.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    sequencer.advanceSequence(completed_2);
    EXPECT_EQ(sequencer.current_seq_index, 3u);
    EXPECT_TRUE(sequencer.isSequenceComplete());
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_NONE);
}

TEST(MissionSequencerTest, MidFlightSequenceUpdate) {
    MissionSequencer sequencer;
    UICommandPacket pkt1{};
    pkt1.sequence_length = 2;
    pkt1.goal_sequence[0] = SHAPE_CIRCLE;
    pkt1.goal_sequence[1] = SHAPE_TRIANGLE;
    sequencer.startMission(pkt1, {0, 0, 1, 0, 0, 0, 0, 1});

    // Advance 1
    sequencer.advanceSequence({2, 0, 1, 0, 0, 0, 0, 1});
    EXPECT_EQ(sequencer.current_seq_index, 1u);

    // Send mid-flight update with 3 new gates
    UICommandPacket pkt2{};
    pkt2.sequence_length = 3;
    pkt2.goal_sequence[0] = SHAPE_SQUARE;
    pkt2.goal_sequence[1] = SHAPE_CIRCLE;
    pkt2.goal_sequence[2] = SHAPE_TRIANGLE;
    sequencer.updateSequence(pkt2, {2, 0, 1, 0, 0, 0, 0, 1});

    EXPECT_EQ(sequencer.current_seq_index, 0u);
    EXPECT_EQ(sequencer.active_sequence_len, 3u);
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_SQUARE);
    EXPECT_TRUE(sequencer.new_goal);
}

TEST(MissionSequencerTest, SequenceLengthClampedToMaxGoalSequenceLen) {
    MissionSequencer sequencer;
    UICommandPacket pkt{};
    pkt.sequence_length = 255; // Deliberate overflow value
    for (size_t i = 0; i < MAX_GOAL_SEQUENCE_LEN; ++i) {
        pkt.goal_sequence[i] = (i % 2 == 0) ? SHAPE_CIRCLE : SHAPE_TRIANGLE;
    }
    pkt.end_action = MISSION_END_LAND;

    Pose start_pose{0, 0, 1, 0, 0, 0, 0, 1};
    sequencer.startMission(pkt, start_pose);

    // active_sequence_len must be clamped to MAX_GOAL_SEQUENCE_LEN (16)
    EXPECT_EQ(sequencer.active_sequence_len, static_cast<uint8_t>(MAX_GOAL_SEQUENCE_LEN));
    EXPECT_FALSE(sequencer.isSequenceComplete());

    // Advance through all 16 items safely
    for (size_t i = 0; i < MAX_GOAL_SEQUENCE_LEN; ++i) {
        EXPECT_EQ(sequencer.current_seq_index, i);
        EXPECT_NE(sequencer.getCurrentTargetShape(), SHAPE_NONE);
        sequencer.advanceSequence({static_cast<double>(i), 0, 1, 0, 0, 0, 0, 1});
    }

    EXPECT_TRUE(sequencer.isSequenceComplete());
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_NONE);

    // Test clamping in updateSequence
    pkt.sequence_length = 32;
    sequencer.updateSequence(pkt, start_pose);
    EXPECT_EQ(sequencer.active_sequence_len, static_cast<uint8_t>(MAX_GOAL_SEQUENCE_LEN));
}

TEST(MissionSequencerTest, EmptySequenceHandling) {
    MissionSequencer sequencer;
    UICommandPacket pkt{};
    pkt.sequence_length = 0; // Empty sequence
    pkt.end_action = MISSION_END_LAND;

    Pose start_pose{0, 0, 1, 0, 0, 0, 0, 1};
    sequencer.startMission(pkt, start_pose);

    EXPECT_EQ(sequencer.active_sequence_len, 0u);
    EXPECT_TRUE(sequencer.isSequenceComplete());
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_NONE);
}

TEST(MissionSequencerTest, OverAdvancePastEndGraceful) {
    MissionSequencer sequencer;
    UICommandPacket pkt{};
    pkt.sequence_length = 1;
    pkt.goal_sequence[0] = SHAPE_CIRCLE;

    Pose start_pose{0, 0, 1, 0, 0, 0, 0, 1};
    sequencer.startMission(pkt, start_pose);
    EXPECT_FALSE(sequencer.isSequenceComplete());

    // Complete sequence
    sequencer.advanceSequence({1, 0, 1, 0, 0, 0, 0, 1});
    EXPECT_TRUE(sequencer.isSequenceComplete());
    EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_NONE);

    // Repeated advances beyond end
    for (int i = 0; i < 5; ++i) {
        sequencer.advanceSequence({2.0 + i, 0, 1, 0, 0, 0, 0, 1});
        EXPECT_TRUE(sequencer.isSequenceComplete());
        EXPECT_EQ(sequencer.getCurrentTargetShape(), SHAPE_NONE);
    }
}
