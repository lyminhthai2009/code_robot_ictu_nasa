#if !defined(ARDUINO) && !defined(ESP_PLATFORM) && !defined(ESP32)
#define WEBOTS_SIM
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }

struct SensorData {
    float ir[4];
    int raw_ir[4];
    int r, g, b;
    float battery_volt;
    double elapsed_sec;
};

enum TrackState {
    STATE_NORMAL_PID = 0,
    STATE_LOOP1_NO_SHORTCUT,
    STATE_SPEED_BUMP,
    STATE_LOOP2_ALIGN,
    STATE_LOOP2_TURN,
    STATE_BRIDGE_CLIMB,
    STATE_SCURVE_ENTRY,
    STATE_FINISH_DECEL
};

class CobotBrain {
public:
    float Kp = 37.0f;
    float Ki = 0.0f;
    float Kd = 4.6f;
    float base_pwm = 232.0f;
    float max_pwm = 255.0f;
    bool loop2_shortcut = true;

    TrackState state = STATE_NORMAL_PID;

    float last_error = 0.0f;
    float filtered_error = 0.0f;
    float filtered_deriv = 0.0f;
    float last_raw_error = 0.0f;
    float curvature_ff = 0.0f;
    float integral = 0.0f;

    int last_line_side = 1;
    int lost_line_frames = 0;
    double lost_line_time = 0.0;
    double off_track_time = 0.0;
    bool was_lifted_off_track = false;
    bool was_near_center = true;
    float prev_outer_max = 0.0f;

    int cross_count = 0;
    int blue_zone_count = 0;
    int red_confirm_frames = 0;

    float est_v_left = 0.0f;
    float est_v_right = 0.0f;
    double total_dist_m = 0.0;
    double total_yaw_rad = 0.0;

    double cross_dist_m = 0.0;
    double cross_yaw_rad = 0.0;
    double max_yaw_excursion = 0.0;
    double state_enter_dist_m = 0.0;
    double state_enter_yaw_rad = 0.0;
    double last_bump_enter_dist_m = -10.0;

    double last_cross_time = -10.0;
    double last_blue_seen_time = -10.0;
    double last_blue_zone_time = -10.0;
    double last_ir0_hit_time = -10.0;
    double last_ir3_hit_time = -10.0;
    double state_enter_time = 0.0;
    double last_bridge_exit_time = -10.0;
    double bridge_exit_dist_m = 0.0;
    double bridge_exit_yaw_rad = 0.0;
    double pitch_up_start_time = -10.0;
    double finish_time_record = -1.0;

    bool shortcut_locked_on = false;
    bool loop2_shortcut_taken = false;
    bool bridge_completed = false;
    bool is_long_bridge = false;
    bool scurve_entry_taken = false;
    bool finished = false;

    CobotBrain() { reset(); }

    void reset() {
        state = STATE_NORMAL_PID;
        last_error = 0.0f;
        filtered_error = 0.0f;
        filtered_deriv = 0.0f;
        last_raw_error = 0.0f;
        curvature_ff = 0.0f;
        integral = 0.0f;
        last_line_side = 1;
        lost_line_frames = 0;
        lost_line_time = 0.0;
        off_track_time = 0.0;
        was_lifted_off_track = false;
        was_near_center = true;
        prev_outer_max = 0.0f;

        cross_count = 0;
        blue_zone_count = 0;
        red_confirm_frames = 0;

        est_v_left = 0.0f;
        est_v_right = 0.0f;
        total_dist_m = 0.0;
        total_yaw_rad = 0.0;
        cross_dist_m = 0.0;
        cross_yaw_rad = 0.0;
        max_yaw_excursion = 0.0;
        state_enter_dist_m = 0.0;
        state_enter_yaw_rad = 0.0;
        last_bump_enter_dist_m = -10.0;

        last_cross_time = -10.0;
        last_blue_seen_time = -10.0;
        last_blue_zone_time = -10.0;
        last_ir0_hit_time = -10.0;
        last_ir3_hit_time = -10.0;
        state_enter_time = 0.0;
        last_bridge_exit_time = -10.0;
        bridge_exit_dist_m = 0.0;
        bridge_exit_yaw_rad = 0.0;
        pitch_up_start_time = -10.0;
        finish_time_record = -1.0;

        shortcut_locked_on = false;
        loop2_shortcut_taken = false;
        bridge_completed = false;
        is_long_bridge = false;
        scurve_entry_taken = false;
        finished = false;
    }

    const char* detectColorTag(int r, int g, int b) const {
        if (r > 70 && r > (int)(g * 1.55f) && r > (int)(b * 1.55f) && (r > g + 30)) return "RED (FINISH)";
        if (b > 75 && r < 80 && g < 105 && b > (int)(r * 1.45f) && b > (int)(g * 1.15f) && (b > r + 22)) return "BLUE (GO / DOC CAU)";
        if (g > 65 && r < 80 && b < 90 && g > (int)(r * 1.45f) && g > (int)(b * 1.25f) && (g > r + 20)) return "GREEN (MARKER / DOC TAN CUONG)";
        if (r < 48 && g < 48 && b < 48) return "BLACK (LINE)";
        return "WHITE (SAN)";
    }

    float computeSmoothLineError(const float s[4], float sum_s) {
        if (state == STATE_BRIDGE_CLIMB) {
            was_near_center = true;
            if (sum_s < 0.18f || (s[0] > 0.65f && s[3] > 0.65f)) {
                return last_error * 0.10f;
            }
        }

        float w[4];
        for (int i = 0; i < 4; i++) {
            w[i] = clampf((s[i] - 0.10f) / 0.90f, 0.0f, 1.0f);
        }

        if (w[0] > 0.20f && w[3] > 0.20f && (w[1] < 0.30f || w[2] < 0.30f)) {
            if (w[1] > w[2] + 0.18f) {
                w[3] = 0.0f;
            } else if (w[2] > w[1] + 0.18f) {
                w[0] = 0.0f;
            } else if (cross_count == 1 || last_line_side < 0) {
                w[3] = 0.0f;
            } else {
                w[0] = 0.0f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        if (w_sum < 0.03f) {
            if (was_near_center && lost_line_time < 0.38) {
                return last_error * 0.15f;
            }
            return (last_line_side >= 0) ? 4.25f : -4.25f;
        }

        float raw_err = (-3.0f * w[0] - 1.0f * w[1] + 1.0f * w[2] + 3.0f * w[3]) / w_sum;

        if (w[3] > 0.02f && w[2] < 0.05f && w[1] < 0.02f && w[0] < 0.02f) {
            float edge_ext = 3.0f + 1.25f * (1.0f - w[3]);
            if (edge_ext > raw_err) raw_err = edge_ext;
        } else if (w[0] > 0.02f && w[1] < 0.05f && w[2] < 0.02f && w[3] < 0.02f) {
            float edge_ext = -3.0f - 1.25f * (1.0f - w[0]);
            if (edge_ext < raw_err) raw_err = edge_ext;
        }

        if (w[3] > 0.22f && w[0] < 0.08f) last_line_side = 1;
        else if (w[0] > 0.22f && w[3] < 0.08f) last_line_side = -1;
        else if (fabsf(raw_err) > 0.80f) last_line_side = (raw_err > 0.0f) ? 1 : -1;

        if ((w[1] > 0.25f || w[2] > 0.25f) && w[0] < 0.18f && w[3] < 0.18f) {
            was_near_center = true;
        } else if (state != STATE_BRIDGE_CLIMB && fabsf(raw_err) > 1.35f) {
            was_near_center = false;
        }

        return clampf(raw_err, -4.25f, 4.25f);
    }

    void updateVirtualOdometry(int pwm_l, int pwm_r, float battery_volt, double dt) {
        if (dt <= 0.0 || dt > 0.1) return;
        float v_max = 0.66f * (battery_volt / 6.0f);
        float cmd_vl = (pwm_l / 255.0f) * v_max;
        float cmd_vr = (pwm_r / 255.0f) * v_max;

        float alpha_m = clampf((float)(dt / (0.020 + dt)), 0.1f, 1.0f);
        est_v_left  += alpha_m * (cmd_vl - est_v_left);
        est_v_right += alpha_m * (cmd_vr - est_v_right);

        double v_center = 0.5 * (double)(est_v_left + est_v_right);
        double omega = (double)(est_v_right - est_v_left) / 0.120;

        if (v_center > 0.0) {
            total_dist_m += v_center * dt;
        }
        total_yaw_rad += omega * dt;

        double yaw_from_last_cross = fabs(total_yaw_rad - cross_yaw_rad);
        if (yaw_from_last_cross > max_yaw_excursion) {
            max_yaw_excursion = yaw_from_last_cross;
        }
    }

    void enterState(TrackState new_state, double now_sec) {
        state = new_state;
        state_enter_time = now_sec;
        state_enter_dist_m = total_dist_m;
        state_enter_yaw_rad = total_yaw_rad;
    }

    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        float sum_ir = s.ir[0] + s.ir[1] + s.ir[2] + s.ir[3];
        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        const char* color_tag = detectColorTag(s.r, s.g, s.b);
        bool is_red   = (strcmp(color_tag, "RED (FINISH)") == 0);
        bool is_blue  = (strcmp(color_tag, "BLUE (GO / DOC CAU)") == 0);
        bool is_green = (strcmp(color_tag, "GREEN (MARKER / DOC TAN CUONG)") == 0);

        if (max_ir < 0.10f && !is_red && !is_blue && !is_green &&
            state != STATE_BRIDGE_CLIMB && state != STATE_LOOP2_TURN && state != STATE_SCURVE_ENTRY) {
            off_track_time += dt;
            if (off_track_time > 0.85) {
                was_lifted_off_track = true;
            }
        } else {
            if (was_lifted_off_track && ((s.ir[1] > 0.35f || s.ir[2] > 0.35f) || is_red || is_green)) {
                reset();
                state_enter_time = s.elapsed_sec;
            }
            off_track_time = 0.0;
        }

        if (s.elapsed_sec >= 150.0 || finished) {
            pwm_left = 0;
            pwm_right = 0;
            return;
        }

        if (max_ir < 0.14f) {
            lost_line_frames++;
            lost_line_time += dt;
        } else {
            lost_line_frames = 0;
            lost_line_time = 0.0;
        }

        if (is_red) red_confirm_frames++;
        else red_confirm_frames = 0;

        if (s.elapsed_sec < 2.0 && cross_count == 0 && total_dist_m < 0.42 &&
            (is_red || is_green || was_near_center) && sum_ir < 0.15f) {
            pwm_left  = (int)max_pwm;
            pwm_right = (int)max_pwm;
            updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
            return;
        }

        double dist_since_cross = total_dist_m - cross_dist_m;
        double dist_since_bridge_exit = total_dist_m - bridge_exit_dist_m;
        double yaw_since_bridge_exit = fabs(total_yaw_rad - bridge_exit_yaw_rad);

        bool finish_topo_ready = (total_dist_m > 1.85 && s.elapsed_sec > 3.5 &&
                                  loop2_shortcut_taken &&
                                  (bridge_completed || dist_since_cross > 1.35) &&
                                  (scurve_entry_taken || (bridge_completed && dist_since_bridge_exit > 0.55 && yaw_since_bridge_exit > 1.20) || dist_since_cross > 1.55));

        if (is_red && (red_confirm_frames >= 1) && finish_topo_ready) {
            if (state != STATE_FINISH_DECEL) {
                enterState(STATE_FINISH_DECEL, s.elapsed_sec);
                printf("[FINISH RED LINE] Reached Red Line at t=%.2f s (dist=%.2f m)! Braking into finish zone...\n",
                       s.elapsed_sec, total_dist_m);
                fflush(stdout);
            }
        }

        bool wide_bar_hit = (s.ir[0] > 0.45f && s.ir[3] > 0.45f && (s.ir[1] > 0.45f || s.ir[2] > 0.45f));
        bool in_bump_corridor = (cross_count == 2);
        bool pre_c_turn_bump = (dist_since_cross > 0.04 && dist_since_cross < 0.45 && max_yaw_excursion < 0.70);
        bool real_bump_bar = in_bump_corridor && (is_blue || (wide_bar_hit && ((s.elapsed_sec - last_blue_seen_time < 0.25) || pre_c_turn_bump)));

        if (real_bump_bar && state != STATE_BRIDGE_CLIMB && state != STATE_FINISH_DECEL) {
            last_blue_seen_time = s.elapsed_sec;
            if (state != STATE_SPEED_BUMP) {
                blue_zone_count++;
                if (total_dist_m - last_bump_enter_dist_m > 0.35) {
                    last_bump_enter_dist_m = total_dist_m;
                }
                enterState(STATE_SPEED_BUMP, s.elapsed_sec);
                printf("[SPEED BUMP ACTIVE] at t=%.2f s! (Color=%d, WideBar=%d)\n",
                       s.elapsed_sec, is_blue, wide_bar_hit);
                fflush(stdout);
            }
        }

        bool on_southern_leg = (loop2_shortcut_taken && cross_count >= 3 && !bridge_completed);

        bool ir_pitch_or_drop = (sum_ir < 0.20f) || wide_bar_hit;
        if (ir_pitch_or_drop) {
            if (pitch_up_start_time < 0.0) pitch_up_start_time = s.elapsed_sec;
        } else {
            pitch_up_start_time = -10.0;
        }

        if (on_southern_leg && state == STATE_NORMAL_PID && dist_since_cross > 0.20) {
            bool color_bridge = is_blue || (is_green && dist_since_cross > 0.25);
            bool optical_pitch_bridge = (pitch_up_start_time > 0.0 &&
                                         (s.elapsed_sec - pitch_up_start_time >= 0.045) &&
                                         was_near_center);
            bool fallback_bridge = (dist_since_cross > 0.25 && dist_since_cross < 0.65 && was_near_center &&
                                    (lost_line_time >= 0.015 || sum_ir < 0.20f || wide_bar_hit));

            if (color_bridge || optical_pitch_bridge || fallback_bridge) {
                is_long_bridge = (dist_since_cross > 0.42);
                enterState(STATE_BRIDGE_CLIMB, s.elapsed_sec);
                printf("[BRIDGE DETECTED] at t=%.2f s (dist_from_X3=%.2f m, long=%d)! (Color=%d, Pitch=%d, Fallback=%d)\n",
                       s.elapsed_sec, dist_since_cross, is_long_bridge, color_bridge, optical_pitch_bridge, fallback_bridge);
                fflush(stdout);
            }
        }

        int max_expected_cross = loop2_shortcut ? 3 : 4;
        if ((state == STATE_NORMAL_PID || state == STATE_SPEED_BUMP) && !loop2_shortcut_taken && cross_count < max_expected_cross) {
            if (s.ir[0] > 0.35f) last_ir0_hit_time = s.elapsed_sec;
            if (s.ir[3] > 0.35f) last_ir3_hit_time = s.elapsed_sec;

            bool x_instant = (s.ir[0] > 0.38f && s.ir[3] > 0.38f && (s.ir[1] > 0.35f || s.ir[2] > 0.35f));
            bool x_outer_sweep = (fabs(last_ir0_hit_time - last_ir3_hit_time) < 0.12) &&
                                 (s.elapsed_sec - last_ir0_hit_time < 0.11) &&
                                 (s.elapsed_sec - last_ir3_hit_time < 0.11);

            bool x_detected = (x_instant || x_outer_sweep) && (s.ir[1] > 0.30f || s.ir[2] > 0.30f || was_near_center);
            bool not_on_blue_bump = !is_blue && (s.elapsed_sec - last_blue_seen_time > 0.12);

            bool topo_gate_open = false;
            if (cross_count == 0) {
                topo_gate_open = (total_dist_m > 0.10 && s.elapsed_sec > 0.20);
            } else if (cross_count == 1) {
                topo_gate_open = (dist_since_cross > 0.28 && max_yaw_excursion > 1.35 &&
                                  (s.elapsed_sec - last_cross_time > 0.55));
            } else if (cross_count == 2) {
                topo_gate_open = (dist_since_cross > 0.40 && max_yaw_excursion > 1.15 &&
                                  (s.elapsed_sec - last_cross_time > 0.85));
            } else if (cross_count == 3) {
                topo_gate_open = (dist_since_cross > 0.28 && max_yaw_excursion > 1.35 &&
                                  (s.elapsed_sec - last_cross_time > 0.55));
            }

            if (x_detected && not_on_blue_bump && topo_gate_open) {
                cross_count++;
                last_cross_time = s.elapsed_sec;
                cross_dist_m = total_dist_m;
                cross_yaw_rad = total_yaw_rad;
                max_yaw_excursion = 0.0;

                printf("[CROSS DETECTED #%d] at t=%.2f s (dist=%.2f m)!\n",
                       cross_count, s.elapsed_sec, total_dist_m);
                fflush(stdout);

                if (cross_count <= 2) {
                    enterState(STATE_LOOP1_NO_SHORTCUT, s.elapsed_sec);
                } else if (cross_count == 3) {
                    if (loop2_shortcut) {
                        enterState(STATE_LOOP2_ALIGN, s.elapsed_sec);
                    } else {
                        enterState(STATE_LOOP1_NO_SHORTCUT, s.elapsed_sec);
                    }
                } else if (cross_count == 4) {
                    loop2_shortcut_taken = true;
                    shortcut_locked_on = true;
                    enterState(STATE_LOOP1_NO_SHORTCUT, s.elapsed_sec);
                }
            }
        }

        double state_duration = s.elapsed_sec - state_enter_time;
        double state_dist = total_dist_m - state_enter_dist_m;

        if (state == STATE_SPEED_BUMP) {
            if ((s.elapsed_sec - last_blue_seen_time > 0.16) || state_duration > 0.48) {
                state = STATE_NORMAL_PID;
            }
        } else if (state == STATE_BRIDGE_CLIMB) {
            bool line_reacquired = (s.ir[1] > 0.30f || s.ir[2] > 0.30f) && (s.ir[0] < 0.40f && s.ir[3] < 0.40f);
            bool color_exit = (!is_blue && !is_green && state_duration >= 0.42 && state_dist >= 0.26 && line_reacquired);
            bool geo_exit   = (state_dist >= (is_long_bridge ? 0.62 : 0.30) && state_duration >= (is_long_bridge ? 0.85 : 0.48) && line_reacquired);
            bool timeout_exit = (state_duration > 1.80);

            if (color_exit || geo_exit || timeout_exit) {
                state = STATE_NORMAL_PID;
                last_bridge_exit_time = s.elapsed_sec;
                bridge_exit_dist_m = total_dist_m;
                bridge_exit_yaw_rad = total_yaw_rad;
                bridge_completed = true;
                was_near_center = true;
                last_error = 0.0f;
                filtered_error = 0.0f;
                curvature_ff = 0.0f;
                printf("[BRIDGE EXIT] Smooth landing at t=%.2f s (dur=%.2f s, bridge_dist=%.2f m)!\n",
                       s.elapsed_sec, state_duration, state_dist);
                fflush(stdout);
            }
        } else if (state == STATE_LOOP1_NO_SHORTCUT) {
            bool cleared_x_wings = (state_dist > 0.026 && state_duration > 0.048 &&
                                    s.ir[0] < 0.28f && s.ir[3] < 0.28f);
            if (cleared_x_wings || state_dist > 0.052 || state_duration > 0.12) {
                state = STATE_NORMAL_PID;
            }
        }

        float voltage_comp = 6.0f / max2f(4.2f, s.battery_volt);

        if (state == STATE_LOOP2_ALIGN) {
            if (state_dist < 0.052 && state_duration < 0.15) {
                float v_adv = 192.0f * voltage_comp;
                pwm_left  = (int)clampf(v_adv, -max_pwm, max_pwm);
                pwm_right = (int)clampf(v_adv, -max_pwm, max_pwm);
                updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
                return;
            } else {
                enterState(STATE_LOOP2_TURN, s.elapsed_sec);
            }
        }

        if (state == STATE_LOOP2_TURN) {
            double turn_dur = s.elapsed_sec - state_enter_time;
            double turn_yaw = fabs(total_yaw_rad - state_enter_yaw_rad);

            if (turn_dur < 0.14 || turn_yaw < 0.85) {
                pwm_left  = (int)clampf( 195.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-125.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
                return;
            }

            if (s.ir[1] > 0.40f || s.ir[2] > 0.40f) {
                loop2_shortcut_taken = true;
                shortcut_locked_on = true;
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
                filtered_error = 0.0f;
                filtered_deriv = 0.0f;
                curvature_ff = 0.0f;
                integral = 0.0f;
                was_near_center = true;
                last_line_side = 1;
                printf("[LOOP2 SHORTCUT LOCKED] at t=%.2f s (turn_dur=%.2f s, yaw=%.2f rad)!\n",
                       s.elapsed_sec, turn_dur, turn_yaw);
                fflush(stdout);
            } else if (turn_dur > 0.48) {
                loop2_shortcut_taken = true;
                shortcut_locked_on = true;
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
                curvature_ff = 0.0f;
            } else {
                pwm_left  = (int)clampf( 185.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-115.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
                return;
            }
        }

        bool scurve_color_trigger = (bridge_completed && !scurve_entry_taken && state == STATE_NORMAL_PID &&
                                     (s.elapsed_sec - last_bridge_exit_time > 0.38) &&
                                     (dist_since_bridge_exit > 0.18 && dist_since_bridge_exit < 1.40) && is_green);
        bool scurve_geo_fallback  = (bridge_completed && !is_long_bridge && !scurve_entry_taken && state == STATE_NORMAL_PID &&
                                     (dist_since_bridge_exit > 0.44 && dist_since_bridge_exit < 0.62) &&
                                     (max_ir < 0.14f || (s.ir[3] > 0.45f && s.ir[0] < 0.10f && s.ir[1] < 0.15f)));

        if (scurve_color_trigger || scurve_geo_fallback) {
            enterState(STATE_SCURVE_ENTRY, s.elapsed_sec);
            printf("[SCURVE ENTRY DETECTED] at t=%.2f s (dist=%.2f m)! Turning right into S-Curve...\n",
                   s.elapsed_sec, total_dist_m);
            fflush(stdout);
        }

        if (state == STATE_SCURVE_ENTRY) {
            double turn_dur = s.elapsed_sec - state_enter_time;
            double turn_yaw = fabs(total_yaw_rad - state_enter_yaw_rad);

            if (turn_dur < 0.22 || turn_yaw < 1.55) {
                pwm_left  = (int)clampf( 190.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-135.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
                return;
            }

            if (s.ir[1] > 0.40f || s.ir[2] > 0.40f || turn_dur > 0.55) {
                scurve_entry_taken = true;
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
                filtered_error = 0.0f;
                filtered_deriv = 0.0f;
                curvature_ff = 0.0f;
                integral = 0.0f;
                was_near_center = true;
                last_line_side = -1;
                printf("[SCURVE ENTRY LOCKED] at t=%.2f s (turn_dur=%.2f s, yaw=%.2f rad)!\n",
                       s.elapsed_sec, turn_dur, turn_yaw);
                fflush(stdout);
            } else {
                pwm_left  = (int)clampf( 180.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-120.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
                return;
            }
        }

        float error = computeSmoothLineError(s.ir, sum_ir);

        float alpha_e = clampf((float)(dt / (0.0042 + dt)), 0.20f, 0.85f);
        float alpha_d = clampf((float)(dt / (0.0120 + dt)), 0.15f, 0.65f);

        filtered_error = (1.0f - alpha_e) * filtered_error + alpha_e * error;
        integral = clampf(integral + filtered_error * (float)dt, -5.0f, 5.0f);

        float raw_deriv = (dt > 0.0001) ? ((error - last_raw_error) / (float)dt) : 0.0f;
        raw_deriv = clampf(raw_deriv, -45.0f, 45.0f);
        filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;
        last_raw_error = error;
        last_error = filtered_error;

        float curv_decay = 1.0f / (1.0f + (float)(dt / 0.195));
        if (state == STATE_NORMAL_PID) {
            if (filtered_error * curvature_ff < 0.0f && fabsf(filtered_error) > 0.5f) {
                curvature_ff *= 0.50f;
            }
            curvature_ff = clampf(curv_decay * curvature_ff + 0.88f * filtered_error * (float)dt, -1.85f, 1.85f);
        } else {
            curvature_ff *= 0.80f;
        }

        float target_base = base_pwm;
        float active_Kp = Kp;
        float active_Kd = Kd;
        float max_turn_clamp = 255.0f;
        float ff_gain = 14.5f;

        if (state == STATE_FINISH_DECEL) {
            double finish_dur = s.elapsed_sec - state_enter_time;
            if (finish_dur > 0.065 || state_dist > 0.042) {
                finished = true;
                pwm_left = 0;
                pwm_right = 0;
                return;
            }
            float ramp = 1.0f - (float)(finish_dur / 0.065);
            target_base = 15.0f + 90.0f * clampf(ramp, 0.0f, 1.0f);
            active_Kp = 22.0f;
            active_Kd = 2.0f;
            ff_gain = 0.0f;
            max_turn_clamp = 35.0f;
        } else if (state == STATE_LOOP1_NO_SHORTCUT) {
            active_Kp = 18.0f;
            active_Kd = 0.8f;
            ff_gain = 0.0f;
            target_base = (cross_count == 1) ? 225.0f : 240.0f;
            max_turn_clamp = 35.0f;
        } else if (state == STATE_SPEED_BUMP) {
            target_base = 205.0f;
            active_Kp = 28.0f;
            active_Kd = 3.0f;
            ff_gain = 0.0f;
            max_turn_clamp = 52.0f;
        } else if (state == STATE_BRIDGE_CLIMB) {
            ff_gain = 0.0f;
            if (is_long_bridge) {
                if (state_dist < 0.42) {
                    target_base = 255.0f;
                    active_Kp = 24.0f;
                    active_Kd = 2.4f;
                    max_turn_clamp = 32.0f;
                } else {
                    target_base = 242.0f;
                    active_Kp = 28.0f;
                    active_Kd = 3.4f;
                    max_turn_clamp = 42.0f;
                }
            } else {
                if (state_dist < 0.22 && state_duration < 0.44) {
                    target_base = 255.0f;
                    active_Kp = 24.0f;
                    active_Kd = 2.2f;
                    max_turn_clamp = 28.0f;
                } else if (state_dist < 0.32 && state_duration < 0.65) {
                    target_base = 205.0f;
                    active_Kp = 28.0f;
                    active_Kd = 3.0f;
                    max_turn_clamp = 45.0f;
                } else {
                    target_base = 165.0f;
                    active_Kp = 32.0f;
                    active_Kd = 4.8f;
                    max_turn_clamp = 45.0f;
                }
            }
        } else {
            float abs_e = fabsf(filtered_error);
            float abs_d = fabsf(filtered_deriv);

            active_Kp = Kp * (1.0f + 0.16f * abs_e);
            active_Kd = Kd * (1.0f + 0.05f * abs_e);

            if (s.elapsed_sec - last_bridge_exit_time < 0.18) {
                target_base = 205.0f;
                max_turn_clamp = 80.0f;
            } else if (bridge_completed && (total_dist_m - bridge_exit_dist_m > 0.35)) {
                if (abs_e < 0.32f && abs_d < 4.5f) {
                    target_base = max_pwm;
                } else if (abs_e < 1.25f) {
                    target_base = clampf(250.0f - 18.0f * abs_e - 0.32f * abs_d, 218.0f, 255.0f);
                } else {
                    target_base = clampf(224.0f - 14.0f * (abs_e - 1.25f) - 0.32f * abs_d, 178.0f, 218.0f);
                }
            } else if (on_southern_leg && dist_since_cross > 0.08 && dist_since_cross < 0.45) {
                target_base = clampf(254.0f - 10.0f * abs_e, 225.0f, 255.0f);
            } else if (abs_e < 0.30f && abs_d < 4.2f) {
                target_base = max2f(base_pwm, 254.0f);
            } else if (abs_e < 1.20f) {
                target_base = 244.0f - 24.0f * abs_e - 0.45f * abs_d;
                target_base = clampf(target_base, 202.0f, 248.0f);
            } else {
                target_base = 212.0f - 17.0f * (abs_e - 1.20f) - 0.38f * abs_d;
                target_base = clampf(target_base, 158.0f, 202.0f);
            }
        }

        float turn_effort = active_Kp * filtered_error + Ki * integral + active_Kd * filtered_deriv + ff_gain * curvature_ff;
        turn_effort = clampf(turn_effort, -max_turn_clamp, max_turn_clamp);

        float v_left = 0.0f, v_right = 0.0f;

        if (off_track_time > 0.85) {
            v_left = 0.0f;
            v_right = 0.0f;
        } else if (max_ir < 0.12f && !was_near_center && lost_line_time >= 0.030 && state == STATE_NORMAL_PID) {
            if (on_southern_leg && !bridge_completed && dist_since_cross > 0.14) {
                v_left  = 180.0f;
                v_right = 180.0f;
            } else {
                if (last_line_side >= 0) {
                    v_left  =  200.0f;
                    v_right = -120.0f;
                } else {
                    v_left  = -120.0f;
                    v_right =  200.0f;
                }
            }
        } else {
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            float abs_err = fabsf(filtered_error);
            float inner_floor = 20.0f;
            if (abs_err >= 1.2f && abs_err < 2.2f) {
                inner_floor = 20.0f - 60.0f * (abs_err - 1.2f);
            } else if (abs_err >= 2.2f) {
                inner_floor = -40.0f - 28.0f * (abs_err - 2.2f);
                inner_floor = max2f(-95.0f, inner_floor);
            }

            if (state != STATE_BRIDGE_CLIMB && state != STATE_SPEED_BUMP) {
                if (v_left < inner_floor)  v_left = inner_floor;
                if (v_right < inner_floor) v_right = inner_floor;
            }
        }

        pwm_left  = (int)clampf(v_left * voltage_comp,  -max_pwm, max_pwm);
        pwm_right = (int)clampf(v_right * voltage_comp, -max_pwm, max_pwm);

        updateVirtualOdometry(pwm_left, pwm_right, s.battery_volt, dt);
    }
};

#ifdef WEBOTS_SIM
#include <string>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

using namespace webots;

static void drawBoldText(Supervisor *robot, int id, const char *text,
                         double x, double y, double font_size, unsigned int color_hex = 0xFF0000) {
    robot->setLabel(id + 1, text, x + 0.0012, y + 0.0015, font_size, 0x000000, 0.35, "Arial Black");
    robot->setLabel(id + 2, text, x + 0.0006, y,          font_size, color_hex, 0.00, "Arial Black");
    robot->setLabel(id + 3, text, x,          y + 0.0006, font_size, color_hex, 0.00, "Arial Black");
    robot->setLabel(id,     text, x,          y,          font_size, color_hex, 0.00, "Arial Black");
}

static void drawBoldRedText(Supervisor *robot, int id, const char *text,
                            double x, double y, double font_size) {
    drawBoldText(robot, id, text, x, y, font_size, 0xFF0000);
}

int main(int argc, char **argv) {
    Supervisor *robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();
    std::string robotName = robot->getName();
    bool isSecondRobot = (robotName == "COBOT_ICTU_2");
    bool isFinalWorld = (robot->getWorldPath().find("chung_ket") != std::string::npos);

    for (int i = 0; i < 100; i++) {
        robot->setLabel(i, "", 0.0, 0.0, 0.0, 0x000000, 0.0);
    }

    Motor *leftMotor = robot->getMotor("left_motor");
    Motor *rightMotor = robot->getMotor("right_motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    DistanceSensor *ir[4];
    const char *ir_names[4] = {"ir0", "ir1", "ir2", "ir3"};
    for (int i = 0; i < 4; i++) {
        ir[i] = robot->getDistanceSensor(ir_names[i]);
        ir[i]->enable(timeStep);
    }
    Camera *colorSensor = robot->getCamera("color_sensor");
    colorSensor->enable(timeStep);

    Keyboard *kb = robot->getKeyboard();
    kb->enable(timeStep);

    Node *lightNode = robot->getFromDef("ENV_LIGHT");
    Field *lightIntensityField = lightNode ? lightNode->getField("intensity") : nullptr;
    Field *lightDirField = lightNode ? lightNode->getField("direction") : nullptr;

    Node *texNode = robot->getFromDef("MAP_TEXTURE");
    Field *texUrlField = texNode ? texNode->getField("url") : nullptr;

    Node *selfNode = robot->getSelf();
    Field *transField = selfNode->getField("translation");
    Field *rotField = selfNode->getField("rotation");

    const double *initTrans = transField->getSFVec3f();
    const double *initRot = rotField->getSFRotation();
    double startPos[3] = {initTrans[0], initTrans[1], initTrans[2]};
    double startRot[4] = {initRot[0], initRot[1], initRot[2], initRot[3]};

    CobotBrain brain;
    double light_intensity = 1.8;
    double light_angle_deg = 0.0;
    int line_width_options[3] = {16, 22, 30};
    int line_width_idx = 1;
    float battery_voltage = 6.0f;
    double match_start_time = 0.0;
    int retry_count = 0;
    bool out_of_line_warning = false;
    int key_cooldown = 0;
    int step_counter = 0;

    while (robot->step(timeStep) != -1) {
        step_counter++;
        double sim_time = robot->getTime();
        double elapsed = sim_time - match_start_time;
        double dt = timeStep / 1000.0;

        battery_voltage = max2f(4.5f, battery_voltage - 0.00004f);

        int key = kb->getKey();
        if (key_cooldown > 0) key_cooldown--;
        if (key != -1 && key_cooldown == 0) {
            key_cooldown = 8;
            if (key == 'L' && !isSecondRobot) {
                light_intensity = (light_intensity + 0.25 > 5.0) ? 5.0 : (light_intensity + 0.25);
                if (lightIntensityField) lightIntensityField->setSFFloat(light_intensity);
            } else if (key == 'K' && !isSecondRobot) {
                light_intensity = (light_intensity - 0.25 < 0.2) ? 0.2 : (light_intensity - 0.25);
                if (lightIntensityField) lightIntensityField->setSFFloat(light_intensity);
            } else if (key == 'J' && !isSecondRobot) {
                light_angle_deg += 25.0;
                double rad = light_angle_deg * M_PI / 180.0;
                double newDir[3] = { -0.6 * cos(rad), -0.6 * sin(rad), -1.0 };
                if (lightDirField) lightDirField->setSFVec3f(newDir);
            } else if (key == 'M' && !isSecondRobot) {
                line_width_idx = (line_width_idx + 1) % 3;
                int w = line_width_options[line_width_idx];
                std::string newUrl = std::string("textures/") + (isFinalWorld ? "vong_chung_ket_" : "vong_bang_") + std::to_string(w) + "mm.png";
                if (texUrlField) texUrlField->setMFString(0, newUrl);
            } else if (key == 'B') {
                battery_voltage = min2f(7.4f, battery_voltage + 0.2f);
            } else if (key == 'V') {
                battery_voltage = max2f(4.5f, battery_voltage - 0.2f);
            } else if (key == 'W') {
                brain.base_pwm = min2f(250.0f, brain.base_pwm + 5.0f);
            } else if (key == 'S') {
                brain.base_pwm = max2f(120.0f, brain.base_pwm - 5.0f);
            } else if (key == 'E') {
                brain.Kp += 2.0f;
            } else if (key == 'Q') {
                brain.Kp = max2f(10.0f, brain.Kp - 2.0f);
            } else if (key == 'D') {
                brain.Kd += 0.2f;
            } else if (key == 'A') {
                brain.Kd = max2f(0.0f, brain.Kd - 0.2f);
            } else if (key == 'T') {
                brain.loop2_shortcut = !brain.loop2_shortcut;
            } else if (key == 'R') {
                transField->setSFVec3f(startPos);
                rotField->setSFRotation(startRot);
                selfNode->resetPhysics();
                brain.reset();
                retry_count++;
            } else if (key == 'N') {
                transField->setSFVec3f(startPos);
                rotField->setSFRotation(startRot);
                selfNode->resetPhysics();
                brain.reset();
                match_start_time = sim_time;
                retry_count = 0;
            }
        }

        SensorData sData;
        sData.battery_volt = battery_voltage;
        sData.elapsed_sec = elapsed;
        float ambient_glare = (float)((light_intensity > 2.2) ? (light_intensity - 2.2) * 0.05 : 0.0);

        const double *curPos = transField->getSFVec3f();
        const double *rotMatrix = selfNode->getOrientation();
        const double ir_y_offsets[4] = { +0.027, +0.009, -0.009, -0.027 };

        int white_sensors = 0;
        for (int i = 0; i < 4; i++) {
            double sensor_x = curPos[0] + rotMatrix[0] * 0.085 + rotMatrix[1] * ir_y_offsets[i] + rotMatrix[2] * (-0.006);
            double sensor_y = curPos[1] + rotMatrix[3] * 0.085 + rotMatrix[4] * ir_y_offsets[i] + rotMatrix[5] * (-0.006);

            bool use_horiz_line = false;
            double target_y = 0.0;
            bool use_vert_line = false;
            double target_x = 0.0;

            if (!isFinalWorld) {
                if (sensor_x > -0.50 && sensor_x < 0.10 && fabs(sensor_y - (-0.620)) < 0.08) {
                    use_horiz_line = true;
                    target_y = -0.620;
                } else if (sensor_x > -0.60 && sensor_x < -0.23 && fabs(sensor_y - 0.620) < 0.06) {
                    use_horiz_line = true;
                    target_y = +0.620;
                } else if (sensor_x > 0.050 && sensor_x < 0.312) {
                    if (fabs(sensor_y - 0.200) < 0.045) { use_horiz_line = true; target_y = +0.200; }
                    else if (fabs(sensor_y + 0.200) < 0.045) { use_horiz_line = true; target_y = -0.200; }
                }
            } else {
                if (sensor_x > 0.265 && sensor_x < 0.575 && fabs(sensor_y - (-0.310)) < 0.08) {
                    use_horiz_line = true;
                    target_y = -0.310;
                } else if (sensor_x > -0.575 && sensor_x < -0.265 && fabs(sensor_y - 0.310) < 0.08) {
                    use_horiz_line = true;
                    target_y = +0.310;
                } else if (fabs(sensor_x) > 0.505 && fabs(sensor_x) < 0.640) {
                    if (fabs(sensor_y - 0.100) < 0.042) { use_horiz_line = true; target_y = +0.100; }
                    else if (fabs(sensor_y + 0.100) < 0.042) { use_horiz_line = true; target_y = -0.100; }
                } else if (fabs(sensor_x) < 0.025 && fabs(sensor_y) < 0.090 && fabs(curPos[0]) < 0.08) {
                    use_vert_line = true;
                    target_x = 0.0;
                }
            }

            if (use_horiz_line || use_vert_line) {
                double dist = use_horiz_line ? fabs(sensor_y - target_y) : fabs(sensor_x - target_x);
                if (dist <= 0.009) {
                    sData.ir[i] = 1.0f;
                    sData.raw_ir[i] = 1024;
                } else if (dist >= 0.017) {
                    sData.ir[i] = 0.07f;
                    sData.raw_ir[i] = 468;
                } else {
                    float frac = (float)((0.017 - dist) / (0.017 - 0.009));
                    sData.ir[i] = 0.07f + 0.93f * frac;
                    sData.raw_ir[i] = (int)(468 + (1024 - 468) * frac);
                }
            } else {
                double val = ir[i]->getValue();
                sData.raw_ir[i] = (int)val;
                float raw_norm = (float)(val / 1024.0) - ambient_glare;
                float calibrated = (raw_norm - 0.42f) / (0.92f - 0.42f);
                sData.ir[i] = clampf(calibrated, 0.0f, 1.0f);
            }
            if (sData.ir[i] < 0.12f) white_sensors++;
        }

        out_of_line_warning = (white_sensors == 4 && !brain.was_near_center);
        (void)out_of_line_warning;

        const unsigned char *img = colorSensor->getImage();
        int width = colorSensor->getWidth();
        sData.r = Camera::imageGetRed(img, width, 1, 1);
        sData.g = Camera::imageGetGreen(img, width, 1, 1);
        sData.b = Camera::imageGetBlue(img, width, 1, 1);

        int pwm_l = 0, pwm_r = 0;
        brain.computeControl(sData, dt, pwm_l, pwm_r);

        char time_hud[64];
        if (brain.finished) {
            if (brain.finish_time_record < 0.0) {
                brain.finish_time_record = elapsed;
                printf("[SUCCESS] %s successfully reached FINISH at t=%.2f s!\n",
                       robotName.c_str(), brain.finish_time_record);
                fflush(stdout);
            }
            snprintf(time_hud, sizeof(time_hud), "HOAN THANH: %.2f s", brain.finish_time_record);

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            if (!isSecondRobot) {
                drawBoldRedText(robot, 0, "BLENDER-ROBLOX STUDIO", 0.02, 0.02, 0.065);
                drawBoldText(robot, 10, time_hud, 0.02, 0.090, 0.055, 0x00FF44);
            } else {
                drawBoldText(robot, 20, "ROBOT 2", 0.65, 0.02, 0.065, 0x0088FF);
                drawBoldText(robot, 30, time_hud, 0.65, 0.090, 0.055, 0x00FF44);
            }
            continue;
        } else {
            snprintf(time_hud, sizeof(time_hud), "THOI GIAN: %.2f s", elapsed);
        }

        double max_omega = 31.416 * (battery_voltage / 6.0);
        double omega_l = (pwm_l / 255.0) * max_omega;
        double omega_r = (pwm_r / 255.0) * max_omega;

        leftMotor->setVelocity((omega_l < -31.416) ? -31.416 : ((omega_l > 31.416) ? 31.416 : omega_l));
        rightMotor->setVelocity((omega_r < -31.416) ? -31.416 : ((omega_r > 31.416) ? 31.416 : omega_r));

        if (step_counter % 6 != 0) continue;

        if (!isSecondRobot) {
            drawBoldRedText(robot, 0, "BLENDER-ROBLOX STUDIO", 0.02, 0.02, 0.065);
            drawBoldText(robot, 10, time_hud, 0.02, 0.090, 0.055, 0xFFD700);
        } else {
            drawBoldText(robot, 20, "ROBOT 2", 0.65, 0.02, 0.065, 0x0088FF);
            drawBoldText(robot, 30, time_hud, 0.65, 0.090, 0.055, 0xFFD700);
        }
    }

    delete robot;
    return 0;
}

#else
#include <Arduino.h>
#include <Wire.h>

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define CH_L_A 0
#define CH_L_B 1
#define CH_R_A 2
#define CH_R_B 3

#define SENSOR_1_PIN 4
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0

const int PIN_IR[4] = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};

#define W_LED_ON  20
#define IR_LED_ON 21

const int MIN_N20_PWM = 36;
float right_motor_trim = 1.00f;
float nominal_battery_volt = 6.0f;

bool INVERT_LEFT_MOTOR  = false;
bool INVERT_RIGHT_MOTOR = false;
bool SWAP_MOTORS_LR     = false;

CobotBrain brain;

int ir_white_cal[4] = {3200, 3200, 3200, 3200};
int ir_black_cal[4] = {1200, 1200, 1200, 1200};
bool ir_white_is_high = true;

const uint8_t TCS34725_ADDR = 0x29;
bool has_i2c_color_sensor = false;
int i2c_fail_streak = 0;

unsigned long startMillis = 0;
unsigned long lastStepMicros = 0;
unsigned long lastColorReadMillis = 0;
unsigned long lastTelemetryMillis = 0;
int cached_r = 255, cached_g = 255, cached_b = 255;
int prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;

void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, 5000, 8, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, 5000, 8, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, 5000, 8, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, 5000, 8, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, 5000, 8);
    ledcSetup(CH_L_B, 5000, 8);
    ledcSetup(CH_R_A, 5000, 8);
    ledcSetup(CH_R_B, 5000, 8);
    ledcAttachPin(PWM_PIN_L_A, CH_L_A);
    ledcAttachPin(PWM_PIN_L_B, CH_L_B);
    ledcAttachPin(PWM_PIN_R_A, CH_R_A);
    ledcAttachPin(PWM_PIN_R_B, CH_R_B);
#endif
}

static inline void writeLEDC(int channel, int pwm) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(channel, pwm);
#elif defined(ESP32)
    ledcWrite(channel, pwm);
#else
    analogWrite(channel, pwm);
#endif
}

static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    if (pwm > 0) return constrain(map(pwm, 1, 255, MIN_N20_PWM, 255), 0, 255);
    return -constrain(map(-pwm, 1, 255, MIN_N20_PWM, 255), 0, 255);
}

static inline int slewLimitPWM(int target, int prev) {
    if ((target > 60 && prev < -60) || (target < -60 && prev > 60)) {
        return 0;
    }
    int delta = target - prev;
    if (delta > 65) return prev + 65;
    if (delta < -90) return prev - 90;
    return target;
}

void setMotorHardware(int pwm_l, int pwm_r) {
    if (brain.finished) {
        prev_hw_pwm_l = 0;
        prev_hw_pwm_r = 0;
        writeLEDC(CH_L_A, 255);
        writeLEDC(CH_L_B, 255);
        writeLEDC(CH_R_A, 255);
        writeLEDC(CH_R_B, 255);
        return;
    }

    if (SWAP_MOTORS_LR) {
        int tmp = pwm_l; pwm_l = pwm_r; pwm_r = tmp;
    }
    if (INVERT_LEFT_MOTOR)  pwm_l = -pwm_l;
    if (INVERT_RIGHT_MOTOR) pwm_r = -pwm_r;

    pwm_l = slewLimitPWM(pwm_l, prev_hw_pwm_l);
    pwm_r = slewLimitPWM(pwm_r, prev_hw_pwm_r);
    prev_hw_pwm_l = pwm_l;
    prev_hw_pwm_r = pwm_r;

    int out_l = applyN20Deadzone(pwm_l);
    int out_r = applyN20Deadzone((int)(pwm_r * right_motor_trim));

    if (out_l >= 0) {
        writeLEDC(CH_L_A, out_l);
        writeLEDC(CH_L_B, 0);
    } else {
        writeLEDC(CH_L_A, 0);
        writeLEDC(CH_L_B, -out_l);
    }

    if (out_r >= 0) {
        writeLEDC(CH_R_A, out_r);
        writeLEDC(CH_R_B, 0);
    } else {
        writeLEDC(CH_R_A, 0);
        writeLEDC(CH_R_B, -out_r);
    }
}

void configureTCS34725Registers() {
    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(0x80 | 0x01); Wire.write(0xFC);
    Wire.endTransmission();

    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(0x80 | 0x0F); Wire.write(0x02);
    Wire.endTransmission();

    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(0x80 | 0x00); Wire.write(0x03);
    Wire.endTransmission();
}

void initColorSensorHardware() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeOut(12);
    Wire.beginTransmission(TCS34725_ADDR);
    if (Wire.endTransmission() == 0) {
        has_i2c_color_sensor = true;
        configureTCS34725Registers();
    }
}

void updateColorSensorHardware(int &r_out, int &g_out, int &b_out) {
    unsigned long nowMs = millis();
    if (nowMs - lastColorReadMillis < 15) {
        r_out = cached_r; g_out = cached_g; b_out = cached_b;
        return;
    }
    lastColorReadMillis = nowMs;

    if (!has_i2c_color_sensor) {
        if (i2c_fail_streak++ >= 20) {
            i2c_fail_streak = 0;
            Wire.beginTransmission(TCS34725_ADDR);
            if (Wire.endTransmission() == 0) {
                has_i2c_color_sensor = true;
                configureTCS34725Registers();
            }
        }
        r_out = 255; g_out = 255; b_out = 255;
        return;
    }

    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(0x80 | 0x14);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(TCS34725_ADDR, (uint8_t)8) == 8) {
        uint16_t c = Wire.read() | (Wire.read() << 8);
        uint16_t r = Wire.read() | (Wire.read() << 8);
        uint16_t g = Wire.read() | (Wire.read() << 8);
        uint16_t b = Wire.read() | (Wire.read() << 8);
        if (c > 8) {
            i2c_fail_streak = 0;
            int nr = (int)((uint32_t)r * 255 / c);
            int ng = (int)((uint32_t)g * 255 / c);
            int nb = (int)((uint32_t)b * 255 / c);
            int avg = (nr + ng + nb) / 3;
            nr = constrain(avg + (nr - avg) * 16 / 10, 0, 255);
            ng = constrain(avg + (ng - avg) * 16 / 10, 0, 255);
            nb = constrain(avg + (nb - avg) * 16 / 10, 0, 255);
            if (c < 55) {
                cached_r = (nr * c) / 120;
                cached_g = (ng * c) / 120;
                cached_b = (nb * c) / 120;
            } else {
                int max_ch = max(nr, max(ng, nb));
                if (max_ch > 0 && max_ch < 140) {
                    cached_r = constrain(nr * 155 / max_ch, 0, 255);
                    cached_g = constrain(ng * 155 / max_ch, 0, 255);
                    cached_b = constrain(nb * 155 / max_ch, 0, 255);
                } else {
                    cached_r = nr;
                    cached_g = ng;
                    cached_b = nb;
                }
            }
        } else {
            if (++i2c_fail_streak >= 3) {
                cached_r = 255; cached_g = 255; cached_b = 255;
                configureTCS34725Registers();
            }
        }
    } else {
        if (++i2c_fail_streak >= 3) {
            cached_r = 255; cached_g = 255; cached_b = 255;
            has_i2c_color_sensor = false;
        }
    }
    r_out = cached_r; g_out = cached_g; b_out = cached_b;
}

static inline int readIRFiltered(int pin) {
    (void)analogRead(pin);
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    int hi = (a > b) ? a : b;
    int lo = (a < b) ? a : b;
    int med = (c > hi) ? hi : ((c < lo) ? lo : c);
    return (med * 2 + a + b + c) / 5;
}

void setup() {
    Serial.begin(115200);

#if defined(ESP32)
    analogReadResolution(12);
#endif

    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, 1);

    pinMode(W_LED_ON, OUTPUT);
    digitalWrite(W_LED_ON, 1);

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IR[i], INPUT);
    }

    setupMotorsHardware();
    setMotorHardware(0, 0);

    initColorSensorHardware();
    delay(80);

    int prev_sample[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) prev_sample[i] = readIRFiltered(PIN_IR[i]);
    for (int wait_iter = 0; wait_iter < 8; wait_iter++) {
        delay(15);
        int max_diff = 0;
        for (int i = 0; i < 4; i++) {
            int cur = readIRFiltered(PIN_IR[i]);
            int diff = abs(cur - prev_sample[i]);
            if (diff > max_diff) max_diff = diff;
            prev_sample[i] = cur;
        }
        if (max_diff < 120) break;
    }

    long sum_raw[4] = {0, 0, 0, 0};
    for (int sample = 0; sample < 32; sample++) {
        for (int i = 0; i < 4; i++) {
            sum_raw[i] += readIRFiltered(PIN_IR[i]);
        }
        delay(3);
    }

    int avg_ir[4];
    for (int i = 0; i < 4; i++) {
        avg_ir[i] = (int)(sum_raw[i] / 32);
    }

    int max_avg = max(max(avg_ir[0], avg_ir[1]), max(avg_ir[2], avg_ir[3]));
    int min_avg = min(min(avg_ir[0], avg_ir[1]), min(avg_ir[2], avg_ir[3]));
    int outer_white = (avg_ir[0] + avg_ir[3]) / 2;
    int inner_line  = (avg_ir[1] + avg_ir[2]) / 2;
    ir_white_is_high = (outer_white >= 2048 || max_avg >= 2350);

    if (ir_white_is_high) {
        int ref_w = max(2100, max(outer_white, max_avg - 120));
        ir_white_cal[0] = max(ref_w, avg_ir[0]);
        ir_white_cal[3] = max(ref_w, avg_ir[3]);
        int mid_w = (ir_white_cal[0] + ir_white_cal[3]) / 2;
        ir_white_cal[1] = max(mid_w, avg_ir[1]);
        ir_white_cal[2] = max(mid_w, avg_ir[2]);

        if (inner_line < mid_w - 350 && inner_line > 120) {
            ir_black_cal[1] = avg_ir[1];
            ir_black_cal[2] = avg_ir[2];
            int mid_b = (avg_ir[1] + avg_ir[2]) / 2;
            ir_black_cal[0] = mid_b;
            ir_black_cal[3] = mid_b;
        } else if (min_avg < mid_w - 400 && min_avg > 120) {
            for (int i = 0; i < 4; i++) ir_black_cal[i] = min_avg;
        } else {
            for (int i = 0; i < 4; i++) ir_black_cal[i] = max(450, ir_white_cal[i] - 1150);
        }
    } else {
        int ref_w = min(1900, min(outer_white, min_avg + 120));
        ir_white_cal[0] = min(ref_w, avg_ir[0]);
        ir_white_cal[3] = min(ref_w, avg_ir[3]);
        int mid_w = (ir_white_cal[0] + ir_white_cal[3]) / 2;
        ir_white_cal[1] = min(mid_w, avg_ir[1]);
        ir_white_cal[2] = min(mid_w, avg_ir[2]);

        if (inner_line > mid_w + 350 && inner_line < 3980) {
            ir_black_cal[1] = avg_ir[1];
            ir_black_cal[2] = avg_ir[2];
            int mid_b = (avg_ir[1] + avg_ir[2]) / 2;
            ir_black_cal[0] = mid_b;
            ir_black_cal[3] = mid_b;
        } else if (max_avg > mid_w + 400 && max_avg < 3980) {
            for (int i = 0; i < 4; i++) ir_black_cal[i] = max_avg;
        } else {
            for (int i = 0; i < 4; i++) ir_black_cal[i] = min(3650, ir_white_cal[i] + 1150);
        }
    }

    Serial.println("====================================================");
    Serial.println(" COBOT ICTU 2026 - VIP PRO ULTIMATE HARDWARE READY ");
    Serial.printf(" TCS34725 I2C : %s\n", has_i2c_color_sensor ? "CONNECTED (16x HIGH-SNR)" : "NOT FOUND (IR FALLBACK ACTIVE)");
    Serial.printf(" IR Polarity  : %s\n", ir_white_is_high ? "WHITE_HIGH (>2048)" : "WHITE_LOW (<2048)");
    Serial.printf(" IR White Cal : [%d, %d, %d, %d]\n", ir_white_cal[0], ir_white_cal[1], ir_white_cal[2], ir_white_cal[3]);
    Serial.printf(" IR Black Cal : [%d, %d, %d, %d]\n", ir_black_cal[0], ir_black_cal[1], ir_black_cal[2], ir_black_cal[3]);
    Serial.println("====================================================");

    brain.reset();
    startMillis = millis();
    lastStepMicros = micros();
}

void loop() {
    unsigned long nowMicros = micros();
    double dt = (nowMicros - lastStepMicros) / 1000000.0;
    if (dt < 0.005) return;
    lastStepMicros = nowMicros;

    SensorData sData;
    sData.elapsed_sec = (millis() - startMillis) / 1000.0;
    float load_factor = (float)(abs(prev_hw_pwm_l) + abs(prev_hw_pwm_r)) / 510.0f;
    float est_batt = nominal_battery_volt - 0.18f * load_factor - 0.002f * (float)sData.elapsed_sec;
    sData.battery_volt = clampf(est_batt, 4.5f, 7.4f);

    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;

        if (brain.state != STATE_BRIDGE_CLIMB && brain.state != STATE_LOOP2_TURN &&
            brain.state != STATE_SCURVE_ENTRY && brain.off_track_time < 0.20) {
            if (ir_white_is_high) {
                if (raw > ir_white_cal[i] && raw < 4040) ir_white_cal[i] = (ir_white_cal[i] * 7 + raw) / 8;
                if (raw < ir_black_cal[i] && raw > 100)  ir_black_cal[i] = (ir_black_cal[i] * 7 + raw) / 8;
            } else {
                if (raw < ir_white_cal[i] && raw > 60)   ir_white_cal[i] = (ir_white_cal[i] * 7 + raw) / 8;
                if (raw > ir_black_cal[i] && raw < 4040) ir_black_cal[i] = (ir_black_cal[i] * 7 + raw) / 8;
            }
        }

        int denom = ir_black_cal[i] - ir_white_cal[i];
        if (abs(denom) < 420) {
            denom = ir_white_is_high ? -420 : 420;
        }

        float norm = (float)(raw - ir_white_cal[i]) / (float)denom;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

    updateColorSensorHardware(sData.r, sData.g, sData.b);

    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 250) {
        lastTelemetryMillis = nowMs;
        int avg_black = (ir_black_cal[0] + ir_black_cal[1] + ir_black_cal[2] + ir_black_cal[3]) / 4;
        int avg_white = (ir_white_cal[0] + ir_white_cal[1] + ir_white_cal[2] + ir_white_cal[3]) / 4;
        for (int i = 0; i < 4; i++) {
            ir_black_cal[i] = (ir_black_cal[i] * 15 + avg_black) / 16;
            ir_white_cal[i] = (ir_white_cal[i] * 15 + avg_white) / 16;
            int span = abs(ir_white_cal[i] - ir_black_cal[i]);
            if (span > 850) {
                if (ir_white_is_high) {
                    ir_white_cal[i] -= 1;
                    ir_black_cal[i] += 1;
                } else {
                    ir_white_cal[i] += 1;
                    ir_black_cal[i] -= 1;
                }
            }
        }
        Serial.printf("[t=%5.2fs] St=%d X=%d Dist=%.2fm | IR=[%.2f,%.2f,%.2f,%.2f] Raw=[%4d,%4d,%4d,%4d] | RGB=(%3d,%3d,%3d) %s | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.state, brain.cross_count, brain.total_dist_m,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3],
                      sData.r, sData.g, sData.b, brain.detectColorTag(sData.r, sData.g, sData.b),
                      pwm_l, pwm_r);
    }
}
#endif
