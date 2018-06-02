#include "PID.h"

#include "common/JSON-Lohmann-2.1.1/json.hpp"
#include "common/format.h"
#include "common/helpers.h"

#include <uWS/uWS.h>
#include <math.h>
#include <iostream>
#include <algorithm>


// For convenience:
using json = nlohmann::json;
using namespace std;


// MAIN:

int main(int argc, char **argv) {

    // PRESETS:
    // -            ALMOST SAFE: 0.15 0 0.0001 0 4 0 0.5 0.5 (CRASH)
    // -                   SAFE: 0.1 0.0005 0.01 0.0001 3 0.015 0.5 0.5 0 0 0 0 0 0 (OK)
    // -                   SAFE: 0.1 0.0005 0.0075 0.0001 3.5 0.01 0.5 0.5 0 0 0 0 0 0 (IMPROVED?)
    // - NOT SO SAFE BUT FASTER: 0.075 0.00025 0.005 0.00015 3.5 0.01 0.75 0 1 0 0.25 0 0.25 0
    // - NOT SO SAFE BUT FASTER: 0.1 0.0005 0.0075 0.0001 3.5 0.01 0.5 0 1 0 0.25 0 0 0 (IMPROVED?)

    // Kp, Lp, Kd, Ld, Ki, Li (STEERING), INTEGRAL_WEIGHT, CONSTANT_THROTTLE, Kp, Lp, Kd, Ld, Ki, Li (THROTTLE):
    const size_t totalParams = (size_t) argc;
    const vector<double> DEF_WEIGHTS = { 0.1, 0.0005, 0.0075, 0.0001, 3.5, 0.01, 0.5, 0.5, 0, 0, 0, 0, 0, 0 };
    vector<double> WEIGHTS;

    for (size_t i = 1; i < totalParams; ++i) {
        WEIGHTS.push_back(atof(argv[i]));
    }

    for (size_t i = totalParams - 1; i < 14; ++i) {
        WEIGHTS.push_back(DEF_WEIGHTS[i]);
    }

    const double INTEGRAL_WEIGHT = WEIGHTS[6];
    const double THROTTLE = WEIGHTS[7];


    // Print them out:

    cout
        << setprecision(6) << fixed
        << endl
        << endl
        << "WEIGHTS STEERING" << endl
        << endl
        << "Kp, Lp = " << WEIGHTS[0] << ", " << WEIGHTS[1] << endl
        << "Ki, Li = " << WEIGHTS[2] << ", " << WEIGHTS[3] << endl
        << "Kd, Ld = " << WEIGHTS[4] << ", " << WEIGHTS[5] << endl
        << "    Wi = " << INTEGRAL_WEIGHT << endl
        << endl;

    if (THROTTLE == 0) {
        cout
            << "WEIGHTS THROTTLE" << endl
            << endl
            << "Kp, Lp = " << WEIGHTS[8] << ", " << WEIGHTS[9] << endl
            << "Ki, Li = " << WEIGHTS[10] << ", " << WEIGHTS[11] << endl
            << "Kd, Ld = " << WEIGHTS[12] << ", " << WEIGHTS[13] << endl
            << endl
            << endl;
    } else {
        cout << "CONSTANT THROTTLE = " << THROTTLE << endl << endl << endl;
    }


    // Init PID controllers:

    PID pid_steer;
    pid_steer.init(WEIGHTS[0], WEIGHTS[1], WEIGHTS[2], WEIGHTS[3], WEIGHTS[4], WEIGHTS[5], INTEGRAL_WEIGHT);

    PID pid_throttle;
    pid_throttle.init(WEIGHTS[8], WEIGHTS[9], WEIGHTS[10], WEIGHTS[11], WEIGHTS[12], WEIGHTS[13], INTEGRAL_WEIGHT);


    // MESSAGE PROCESSING:

    uWS::Hub h; // Initialize WebSocket.

    h.onMessage([
        &THROTTLE,
        &pid_steer,
        &pid_throttle
    ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event
        const string sdata = string(data).substr(0, length);

        if (sdata.size() <= 2 || sdata[0] != '4' || sdata[1] != '2') {
            return;
        }

        const string s = helpers::hasData(sdata);

        if (s == "") {
            // Manual driving:

            string msg = "42[\"manual\",{}]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            return;
        }

        const auto j = json::parse(s);
        const string event = j[0].get<string>();

        if (event != "telemetry") {
            return;
        }

        // j[1] is the data JSON object:

        const double CTE = stod(j[1]["cte"].get<string>());
        const double speed = stod(j[1]["speed"].get<string>());
        const double angle = stod(j[1]["steering_angle"].get<string>());

        // Calculate steer value:
        double steer_value = 0.8 * pid_steer.update(CTE, 1.5 * speed, -1, 1) - 0.2 * angle/25;

        // Calculate throttle (unless set to a fixed value):

        double throttle = THROTTLE;

        if (throttle == 0) {
            const double maxSpeed = 100;
            const double minSpeed = 30;
            const double targetSpeed = maxSpeed - min(abs(CTE), 2.0) * (maxSpeed - minSpeed) / 2;

            // This min(0, diff) makes the throttle response asymmetric: accelerate hard but break soft.
            // If the min is set to 0, the car will never break, it will just release the throttle.
            // Otherwise, if set to anything > 0, the car will break softer that it would without this.
            throttle = pid_throttle.update(min(0.0, speed - targetSpeed), CTE, -1, 1);
        }

        // Emergency breaking:

        /*
        if ( abs(CTE) > 1.5 && speed > 50 ) {
            cout << "BRAKE!" << endl;
            throttle = -0.5;
        }
        */

        // DEBUG

        cout
            << setprecision(6) << fixed
            << setw(10) << CTE << " │ "
            << setw(10) << pid_steer.err_p_ << " │ "
            << setw(10) << pid_steer.err_i_ << " │ "
            << setw(10) << pid_steer.err_d_ << " │ "
            << setw(10) << pid_steer.err_total_ << " │ "
            << setw(10) << steer_value
            << endl;

        // Send message back to the emulator:

        json msgJson;

        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = throttle;

        const string msg = "42[\"steer\"," + msgJson.dump() + "]";

        // Send it:
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // ON HTTP REQUEST:
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(

    h.onHttpRequest([](
        uWS::HttpResponse *res,
        uWS::HttpRequest req,
        char *data,
        size_t,
        size_t
     ) {
        const string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    // ON CONNECTION:

    h.onConnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        uWS::HttpRequest req
     ) {
        cout
            << endl
            << " Connected!" << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl
            << endl;
    });

    // ON DISCONNECTION:

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        int code,
        char *message,
        size_t length
     ) {
        ws.close();

        cout
            << endl
            << "Disconnected!" << endl
            << endl
            << SEPARATOR << endl;
    });

    // START LISTENING:

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl;

    } else {

        cerr
            << endl
            << "Failed to listen on port" << port << "!"
            << endl
            << SEPARATOR << endl;

        return -1;
    }

    h.run();
}
