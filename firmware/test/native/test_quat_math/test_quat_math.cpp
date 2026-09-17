// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martin Pollow

#include <unity.h>
#include <math.h>
#include "quat_math.h"

void setUp(void) {}
void tearDown(void) {}

static const float EPS = 1e-5f;

void test_identity_multiply_is_identity(void) {
    Quat id = {1, 0, 0, 0};
    Quat q  = {0.5f, 0.5f, 0.5f, 0.5f};   // 120° about (1,1,1)
    Quat r  = quat_mul(id, q);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.w, r.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.x, r.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.y, r.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.z, r.z);
}

void test_conjugate_inverts_unit_quat(void) {
    Quat q = {0.5f, 0.5f, 0.5f, 0.5f};
    Quat r = quat_mul(quat_conj(q), q);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, r.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, r.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, r.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, r.z);
}

void test_rezero_against_self_is_identity(void) {
    Quat raw = {0.7071068f, 0.0f, 0.7071068f, 0.0f};   // 90° about Y
    Quat applied = quat_mul(raw, quat_conj(raw));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, applied.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, applied.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, applied.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, applied.z);
}

// Yaw of 90° about world Z:  q = (cos(45°), 0, 0, sin(45°))
void test_to_euler_zyx_yaw_90(void) {
    Quat q = {0.7071068f, 0.0f, 0.0f, 0.7071068f};
    Euler e = quat_to_euler(q);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, M_PI_2, e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,   e.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,   e.roll);
}

// Pitch of 30° about Y in the ZYX convention.
void test_to_euler_zyx_pitch_30(void) {
    float a = (float)(30.0 * M_PI / 180.0 / 2.0);
    Quat q = {cosf(a), 0, sinf(a), 0};
    Euler e = quat_to_euler(q);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,                   e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(30.0*M_PI/180), e.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,                   e.roll);
}

// Sensor mounted with chip-X pointing world-Z (side mount, Ry(-90°) at neutral).
// A world yaw of 90° must appear as yaw=90°, not as roll.
void test_rezero_side_mount_yaw_decoupled(void) {
    const float sq2 = 0.7071068f;
    Quat q_neutral = {sq2, 0.0f, -sq2, 0.0f};          // Ry(-90°): chip-X → world-Z
    Quat q_yaw90   = {sq2, 0.0f,  0.0f, sq2};           // Rz(+90°): world yaw
    Quat q_raw     = quat_mul(q_yaw90, q_neutral);      // sensor reads yaw on top of mount
    Quat q_rel     = quat_mul(q_raw, quat_conj(q_neutral));
    Euler e = quat_to_euler(q_rel);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)M_PI_2, e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, e.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, e.roll);
}

// Sensor mounted at Rz(+90°). After applying the mount correction, a further
// 180° world yaw should appear as 180° yaw (not a compound rotation).
void test_mount_quat_removes_mounting_offset(void) {
    const float sq2 = 0.7071068f;
    Quat q_mount = {sq2, 0.0f, 0.0f, sq2};    // Rz(+90°): sensor yaw-rotated at neutral
    Quat q_extra = {0.0f, 0.0f, 0.0f, 1.0f};  // Rz(+180°): device then yaws 180° more
    Quat q_raw = quat_mul(q_extra, q_mount);
    Quat q_corrected = quat_mul(q_raw, quat_conj(q_mount));
    Euler e = quat_to_euler(q_corrected);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)M_PI, e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, e.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, e.roll);
}

// Sensor mounted at Rz(+90°). After rezero the output must be identity, and a
// subsequent delta rotation must pass through unchanged.
void test_rezero_with_mount_quat_gives_identity(void) {
    const float sq2 = 0.7071068f;
    Quat q_mount    = {sq2, 0.0f, 0.0f, sq2};   // Rz(+90°) mounting correction
    Quat q_raw      = {0.3f, 0.1f, -0.2f, sq2}; // arbitrary pose at rezero moment

    // Normalise q_raw so it is a unit quaternion.
    float len = sqrtf(q_raw.w*q_raw.w + q_raw.x*q_raw.x + q_raw.y*q_raw.y + q_raw.z*q_raw.z);
    q_raw.w /= len; q_raw.x /= len; q_raw.y /= len; q_raw.z /= len;

    // Replicate the fixed rezero_to() logic: offset = raw * mount
    Quat offset = quat_mul(q_raw, q_mount);

    // At rezero moment the pipeline gives identity.
    Quat rotated    = quat_mul(quat_mul(q_raw, q_mount), quat_conj(offset));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rotated.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, rotated.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, rotated.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, rotated.z);

    // A further Rz(+45°) delta must appear unchanged in the output.
    Quat delta      = {sq2, 0.0f, 0.0f, sq2 * 0.4142136f}; // approx Rz(+45°)
    float dlen = sqrtf(delta.w*delta.w + delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    delta.w /= dlen; delta.x /= dlen; delta.y /= dlen; delta.z /= dlen;
    Quat q_after    = quat_mul(delta, q_raw);
    Quat rotated2   = quat_mul(quat_mul(q_after, q_mount), quat_conj(offset));
    TEST_ASSERT_FLOAT_WITHIN(EPS, delta.w, rotated2.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, delta.x, rotated2.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, delta.y, rotated2.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, delta.z, rotated2.z);
}

// Coupled mount (yaw AND pitch nonzero): the correction must be the full inverse
// conj(P), which negates the angles AND reverses the ZYX order. Mirrors the
// firmware's mount_correction_from_ypr. Naively negating the three angles in
// place (same ZYX order) is NOT the inverse once axes are coupled and fails to
// recover the head orientation — this test pins that down.
void test_coupled_mount_correction_is_conjugate(void) {
    const float d2r = (float)(M_PI / 180.0);
    Euler ypr{ 90.0f * d2r, 30.0f * d2r, 0.0f };   // physical sensor displacement
    Quat P = euler_zyx_to_quat(ypr);

    Quat q_extra = {0.6f, 0.1f, -0.3f, 0.7f};      // arbitrary head pose
    float len = sqrtf(q_extra.w*q_extra.w + q_extra.x*q_extra.x
                    + q_extra.y*q_extra.y + q_extra.z*q_extra.z);
    q_extra.w/=len; q_extra.x/=len; q_extra.y/=len; q_extra.z/=len;

    Quat q_raw = quat_mul(q_extra, P);             // sensor reads head motion on top of mount

    // Correcting with conj(P) recovers the head pose exactly.
    Quat corrected = quat_mul(q_raw, quat_conj(P));
    TEST_ASSERT_FLOAT_WITHIN(EPS, q_extra.w, corrected.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q_extra.x, corrected.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q_extra.y, corrected.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q_extra.z, corrected.z);

    // Naive per-angle negation (same ZYX order) must NOT recover the head pose.
    Euler neg{ -90.0f * d2r, -30.0f * d2r, 0.0f };
    Quat naive = quat_mul(q_raw, euler_zyx_to_quat(neg));
    bool differs = fabsf(naive.w - q_extra.w) > 1e-3f
                || fabsf(naive.x - q_extra.x) > 1e-3f
                || fabsf(naive.y - q_extra.y) > 1e-3f
                || fabsf(naive.z - q_extra.z) > 1e-3f;
    TEST_ASSERT_TRUE(differs);
}

// quat_canonical_to: passes q through when in the same hemisphere as ref,
// negates otherwise. Models the BNO085 double-cover sign flip.
void test_canonical_to_same_hemisphere_passes_through(void) {
    Quat ref = {0.9f, 0.1f, 0.2f, 0.3f};
    Quat q   = {0.8f, 0.2f, 0.1f, 0.4f};   // dot(q,ref) > 0
    Quat r   = quat_canonical_to(q, ref);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.w, r.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.x, r.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.y, r.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, q.z, r.z);
}

void test_canonical_to_opposite_hemisphere_negates(void) {
    // Real numbers from the user's CDC trace: BNO085 jumped from
    // (+0.8822, -0.1074, +0.1815, -0.4210) to (-0.8489, +0.1265, -0.2025, +0.4716)
    // mid-rotation. After canonicalisation against the previous sample, the
    // second quaternion must be flipped to its +q representation so the stream
    // stays continuous.
    Quat prev = {+0.8822f, -0.1074f, +0.1815f, -0.4210f};
    Quat raw  = {-0.8489f, +0.1265f, -0.2025f, +0.4716f};
    Quat r    = quat_canonical_to(raw, prev);
    TEST_ASSERT_FLOAT_WITHIN(EPS, +0.8489f, r.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.1265f, r.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, +0.2025f, r.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.4716f, r.z);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_identity_multiply_is_identity);
    RUN_TEST(test_conjugate_inverts_unit_quat);
    RUN_TEST(test_rezero_against_self_is_identity);
    RUN_TEST(test_rezero_side_mount_yaw_decoupled);
    RUN_TEST(test_to_euler_zyx_yaw_90);
    RUN_TEST(test_to_euler_zyx_pitch_30);
    RUN_TEST(test_mount_quat_removes_mounting_offset);
    RUN_TEST(test_rezero_with_mount_quat_gives_identity);
    RUN_TEST(test_coupled_mount_correction_is_conjugate);
    RUN_TEST(test_canonical_to_same_hemisphere_passes_through);
    RUN_TEST(test_canonical_to_opposite_hemisphere_negates);
    return UNITY_END();
}
