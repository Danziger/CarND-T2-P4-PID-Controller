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

int main() {

    PID pid_steer;

    // pid.init(0.15, 0.0001, 3.5);
    pid_steer.init(0.15, 0.0001, 4);

    PID pid_throttle;

    pid_throttle.init(0, 0.001, -10);

    // MESSAGE PROCESSING:

    uWS::Hub h; // Initialize WebSocket.

    h.onMessage([
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

        double cte = stod(j[1]["cte"].get<string>());
        double speed = stod(j[1]["speed"].get<string>());
        double angle = stod(j[1]["steering_angle"].get<string>());

        /*
        * TODO: Calcuate steering value here, remember the steering value is
        * [-1, 1].
        * NOTE: Feel free to play around with the throttle and speed. Maybe use
        * another PID controller to control the speed!
        */

        const double steer_value = pid_steer.update(cte, -1, 1);

        // double throttle = pid_throttle.update(speed - 50 + 20 * cte/7, -1, 1);
        // double throttle = pid_throttle.update((speed - 50) + 30 * cte/3, -1, 1);
        double throttle = pid_throttle.update(min(0.0, speed - 50) + 50 * cte / 3, -1, 1);

        /*
        double throttle = 1;

        if (cte > 1.5) {
            throttle = -cte + 2;
        } else if (cte < -1.5) {
            throttle = cte + 2;
        }
        */

        // DEBUG
        cout << "CTE: " << cte << " Steering Value: " << steer_value << endl;

        // Send message back to the emulator:

        json msgJson;

        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = speed < 50 ? 1 : throttle;

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
