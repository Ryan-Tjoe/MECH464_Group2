#include "teleop/comms.h"
#include <sys/socket.h>

using nlohmann::json;
namespace teleop
{
    Comms::Comms()
    {
        sensorOffsetSkew << 0, -55, 35,
                            55, 0, 0,
                            -35, 0, 0; // millimetres

        // For haptic device facing same direction as robot
        Matrix3d R; // or switch the signs on x and y
        R <<  0, -1, 0,
              1, 0, 0,
              0, 0, 1;
        slave_T_master = slave_T_master.rotate(R);
        master_T_slave = slave_T_master.inverse();

        R << 0, 0, 1,
             1, 0, 0,
             0, 1, 0;
        flange_T_ee = flange_T_ee.rotate(R);
        flange_T_ee = flange_T_ee.pretranslate(Vector3d(0,0,0.17));
        ee_T_flange = flange_T_ee.inverse();
    }

    Comms::~Comms()
    {
        cleanUp();
    }

    void Comms::initComms(const std::string& id, const int delay, bool peerIsImfusion) {
        localId = id + "_ehe";
        remoteId = id + "_ehh";

        initComms(delay, peerIsImfusion);
    }

    void Comms::initComms(const int delay, bool peerIsImfusion) {
        m_peerIsImFusion = peerIsImfusion;
        goalDelay = delay;
        try {
            rtc::InitLogger(rtc::LogLevel::Error);
            if (!websocketOpen)
                createWebSocket();
        }
        catch (const std::exception &e) {
            LOG << localId << ": " << "Error creating comms: " << e.what();
        }
        createForceSensorSocket();
    }

    void Comms::commThreadLoop()
    {
        while (isalive)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Check for general messages (don't delay these)
            if (RtcMessage sendMsg; sendMsgQ.try_dequeue(sendMsg))
                SendMsg(sendMsg.channel, sendMsg.message);

            // Check for state feedback
            // Check if we're adding a delay and only delay the state feedback
            if (delayMs > 0) {
                // When initially turned on, ramp the delay gradually to avoid a jump
                if (delayMs < goalDelay) {
                    delayDouble += delayRamp;
                    delayMs = (int)round(delayDouble);
                }
                auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                if (auto* msg = sendStateQ.peek()) {
                    if (t - msg->timestamp < delayMs)
                        continue;
                } else continue;
            }

            if (RtcMessage sndMsg; sendStateQ.try_dequeue(sndMsg))
                SendMsg(sndMsg.channel, sndMsg.message);
        }
    }

    void Comms::SendSdp(const std::string& id, const std::string& data, const std::string& msgType, std::weak_ptr<rtc::WebSocket> wws) {
        json message;

        if (m_peerIsImFusion) {
            message = { {"version", "2.0"},
                        {"name", ""},
                        {"propertyfile", { {"id", localId},
                                           {"messageType", pc->localDescription()->typeString()},
                                           {"data", pc->localDescription().value()}}
                        }
                      };
        } else {
            message = { {"Id", localId},
                        {"MessageType", pc->localDescription()->typeString()},
                        {"Data", pc->localDescription().value()},
                        {"IceDataSeparator", "|"}};
        }

        if (auto wss = wws.lock())
            wss->send(message.dump());
    }

    void Comms::createPeerConnection(const rtc::Configuration &config, std::weak_ptr<rtc::WebSocket> wws) { //NOLINT
        this->pc = std::make_shared<rtc::PeerConnection>(config);
        pc->onStateChange(
            [this](rtc::PeerConnection::State state) {
                if (state == rtc::PeerConnection::State::Connected){
                    LOG << remoteId << " Connected";
                    isalive = true;
                    commThread = std::thread([this](){commThreadLoop();});
                }
                else if (state == rtc::PeerConnection::State::Connecting){
                    LOG << remoteId << " Connecting";
                }
                else if (state == rtc::PeerConnection::State::Disconnected) {
                    LOG << remoteId << " Disconnected";
                } else if(state == rtc::PeerConnection::State::Closed){
                    if(isalive){
                        LOG << remoteId << " Closed";
                        isalive = false;
                        commThread.join();

                        // Restart communication
                        cleanUp(false);
                        initComms(goalDelay);
                    }

                }
            });

        pc->onGatheringStateChange([this, wws](rtc::PeerConnection::GatheringState state) {
            LOG << localId << ": " << "Gathering State: " << static_cast<int>(state);
            if (state == rtc::PeerConnection::GatheringState::Complete) {
                json message = { {"id", localId},
                                {"messageType", pc->localDescription()->typeString()},
                                {"data", pc->localDescription().value()}};
                if (auto wss = wws.lock())
                    wss->send(message.dump());
            }
        });
        pc->onLocalDescription([this, wws](const rtc::Description& description) {
            if (!useTrickleIce) return;

            json message = { {"id", localId},
                            {"messageType", description.typeString()},
                            {"data", std::string(description)}};

            LOG << "Sending local description";
            if (auto wss = wws.lock())
                wss->send(message.dump());
        });
        pc->onLocalCandidate([this, wws](const rtc::Candidate& candidate) {
            if (!useTrickleIce) return;

            std::string content = std::string(candidate) + iceDataSeparator + candidate.mid() + iceDataSeparator + candidate.mid();
            json message = { {"id", localId},
                             {"messageType", "candidate"},
                             {"data", content}};

            LOG << "Sending local candidate";
            if (auto wss = wws.lock())
                wss->send(message.dump());
        });

        pc->onDataChannel([this](std::shared_ptr<rtc::DataChannel> dc) { //NOLINT
            LOG << (localId) << ": " << "DataChannel from " << (remoteId) << " received with label " << (dc->label());

            dc->onOpen([this, wdc = make_weak_ptr(dc)]() {
                if (const auto ddc = wdc.lock())
                    LOG << (localId) << ": "<< "DataChannel " << (ddc->label()) << " from " << (remoteId) << " opened";
            });

            if (dc->label() == "state") {
                setupStateChannel(dc);
            } else if (dc->label() == "teleop") {
                setupTeleopChannel(dc);
            } else if (dc->label() == "RtcControl") {
                setupControlChannel(dc);
            } else if (dc->label() == "time") {
                setupTimingChannel(dc);
            } else if (dc->label() == "aruco") {
                setupArucoChannel(dc);
            } else {
                dc->onClosed([this, wdc = make_weak_ptr(dc)]() {
                    if (const auto ddc = wdc.lock())
                        LOG << (localId) << ": " << "DataChannel " << (ddc->label()) << " from " << (remoteId) << " closed";
                });

                dc->onMessage([this, wdc = make_weak_ptr(dc)](auto data) {
                    if (const auto ddc = wdc.lock())
                        LOG << (localId) << ": " << "DataChannel " << (ddc->label()) << " received message";
                });
            }

            {
                std::lock_guard<std::mutex> lock(dataChannelMapMutex); // Protect dataChannelMap access
                this->dataChannelMap.emplace(dc->label(), dc);
            }
        });

        pc->onTrack([this](const std::shared_ptr<rtc::Track>& track) {

            track->onOpen([this]() {
                LOG << (localId) << ": " << "Track from " << (remoteId) << " open";
            });

            track->onClosed([this]() {
                LOG << (localId) << ": " << "Track from " << (remoteId) << " closed";
            });

            // Deal with audio track
            if (track->description().type() == "audio") {
                LOG << (localId) << ": " << "Received audio track";

                track->onMessage(
                    [](const rtc::binary& message) { //session, addr,
                        std::cout << "Received audio message" << std::endl;
                    },
                    nullptr);
            }
            // Deal with video track
            else if (track->description().type() == "video" && track->direction() == rtc::Description::Direction::RecvOnly) {
                LOG << (localId) << ": " << "Received video track called " << (track->mid());
                if (track->mid() == "vido")
                    setupVideoTrack(track);
                else if (track->mid() == "dpth")
                    setupDepthTrack(track);
                else LOG << "Unknown video track";
            }
            // Ignore other tracks
            else  {
                // Don't know what to do with this
                LOG << (localId) << ": " << "Received " << (track->description().type()) << " track which will be ignored";
                track->onMessage([](const rtc::binary& message) {
                    //LOG << (localId) << ": "<< "Received frame on unknown media track";
                }, nullptr);
            }
        });
    };

    void Comms::bytes2floats(const std::vector<std::byte>& d, float* vec, const uint length) { //NOLINT
        // Then copy them into a float
        memcpy(vec, d.data(), length*sizeof(float));
    }

    std::string Comms::string_format( const std::string& format, float val )
    {
        int size_s = std::snprintf( nullptr, 0, format.c_str(), val ) + 1; // Extra space for '\0'
        if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        auto size = static_cast<size_t>( size_s );
        std::unique_ptr<char[]> buf( new char[ size ] );
        std::snprintf( buf.get(), size, format.c_str(), val );
        return { buf.get(), buf.get() + size - 1 }; // We don't want the '\0' inside
    }

    std::vector<std::string> Comms::strSplit(const std::string& s, const std::string& delim)
    {
        std::vector<std::string> ret;

        size_t idx = s.find(delim);
        size_t idxLast = 0;
        while (idx != std::string::npos)
        {
            ret.push_back(s.substr(idxLast, idx-idxLast));

            if (idx >= s.size() - 1) break;

            idxLast = idx + 1;
            idx = s.find(delim, idxLast);
        }
        ret.push_back(s.substr(idxLast, idx-idxLast));

        return ret;
    }

    void Comms::createWebSocket() {
        // Set up WebSocket for signaling
        ws = std::make_shared<rtc::WebSocket>();

        // WebSocket callbacks
        ws->onOpen([this]() {
            LOG << "WebSocket open";
            ws->send("WAK4k5SthTsWrAxp49U4yfybjpjZ7XRu" + localId);
            LOG << (localId) << ": " << "WebSocket connected, signaling ready";
            websocketOpen = true;
        });

        ws->onError([this](const std::string& s) {
            LOG << (localId) << ": " << "WebSocket error";
        });

        ws->onClosed([this]() {
            LOG << "WebSocket closed";
            websocketOpen = false;
        });

        ws->onMessage([this](auto data) {
            // LOG << "Got message";
            // data holds either std::string or rtc::binary
            std::string msg;
            rtc::binary bts;
            if (!std::holds_alternative<std::string>(data)) {
                LOG << "Received bytes instead of string";
            }
            else
                msg = std::get<std::string>(data);

           // Look for special messages
            std::string spec = msg.substr(0,4);
            if (spec == "***") {
                return;
            }
            if (spec == "PING") {
                ws->send("PONG");
                return;
            }
            if (spec == "@@@") {
                // Peer is present. We let the peer initiate, so who cares
                LOG << (localId) << ": " << "Peer arrived on signaling server";
                return;
            }
            if (spec == "###") {
                LOG << (localId) << ": " << "Peer left signaling server";
                return;
            }

            // Now parse it
            json message;
            try {
                if (m_peerIsImFusion) {
                    json prop = json::parse(msg);
                    if (!prop.contains("propertyfile")) {
                        LOG << "Received JSON string with unexpected format";
                        return;
                    }

                    message = prop["propertyfile"];
                } else 
                    message = json::parse(msg);
            } catch (const json::exception&) {
                LOG << "Could not parse JSON message: " << (msg);
                return;
            }

            // Get sender ID
            auto it = message.find("id");
            if (it == message.end())
                return;
            auto id = it->get<std::string>();

            // Get message type
            it = message.find("messageType");
            if (it == message.end())
                return;
            std::string type = it->get<std::string>();

            if (type == "offer") {
                try {
                    rtc::Configuration config;
                    config.iceServers.emplace_back(stunServer);
                    for (const auto & turnUrl : turnUrls) {
                        rtc::IceServer ic(turnUrl);
                        ic.password = turnPassword;
                        ic.username = turnUsername;
                        config.iceServers.push_back(ic);
                    }
                    config.disableAutoNegotiation = true;
                    createPeerConnection(config, make_weak_ptr(ws));
                } catch (const std::exception&) {
                    return;
                }
            }
            else {
                return;
            }

            // Handle the message
            if (type == "offer") {
                auto sdp = message["data"].get<std::string>();
                this->pc->setRemoteDescription(rtc::Description(sdp, "offer"));
                this->pc->setLocalDescription();
            }
            else if (type == "answer") {
                auto sdp = message["data"].get<std::string>();
                this->pc->setRemoteDescription(rtc::Description(sdp, "answer"));
            }
            else if (type == "candidate") {
                auto mesg = message["data"].get<std::string>();
                size_t pos = mesg.find(iceDataSeparator);
                size_t pos2 = mesg.substr(pos+1).find(iceDataSeparator);
                auto sdp = mesg.substr(0, pos);
                auto sdpMLineIdx = mesg.substr(pos+1,pos2);
                auto mid = mesg.substr(pos2+1);

                this->pc->addRemoteCandidate(rtc::Candidate(sdp, mid));
            }
        });

        ws->open(this->wsUrl);
        LOG << "Started websocket";
    }

    void Comms::createForceSensorSocket() {
        // Set up WebSocket server for force sensor to connect to. Use default values (localhost, port 8080, no encryption, etc.)
        sensorServer = std::make_shared<rtc::WebSocketServer>();

        sensorServer->onClient([this](const std::shared_ptr<rtc::WebSocket>& wss){
            wss->onOpen([this](){
                LOG << "Force sensor client connected";
                hasSensor = true;
            });

            wss->onError([this](const std::string& s) {
                LOG << "Force sensor sensor client error: " << s;
            });

            wss->onClosed([this](){
                LOG << "Force sensor sensor client closed";
                hasSensor = false;
            });

            wss->onMessage([this](auto data){
                // data holds either std::string or rtc::binary
                std::string msg;
                if (!std::holds_alternative<std::string>(data)) {
                    LOG << "Received bytes instead of string";
                }
                else
                    msg = std::get<std::string>(data);

                // Parse the wrench from string
                const auto split = strSplit(msg, ",");
                if (split.size() != 6) return;
                Vector6d wrench; int i = 0;
                for (const auto& s : split) {
                    wrench[i] = std::stod(s);
                    i++;
                }

                // Transform to the correct coordinates
                double tmp = wrench[0];
                wrench[0] = -wrench[1];
                wrench[1] = tmp;
                tmp = wrench[3];
                wrench[3] = -wrench[4];
                wrench[4] = tmp;

                // Remove the wrench that is due to the offset in sensor position from the end-effector
                wrench.tail(3) << (Matrix3d::Identity() - sensorOffsetSkew) * wrench.head(3);

                wrenchQ.emplace(wrench);
            });

            sensorClients.push_back(wss);
        });

        LOG << "Started Force Sensor WebSocket server";
    }
    

    void Comms::setupVideoTrack(const std::shared_ptr<rtc::Track>& track) {
        // Set up media pipeline
        const auto mediaHandler = std::make_shared<rtc::H264RtpDepacketizer>(rtc::H264RtpDepacketizer::Separator::LongStartSequence);
        mediaHandler->addToChain(std::make_shared<rtc::RtcpReceivingSession>());
        track->setMediaHandler(mediaHandler);

        // Set up video receiving
        track->onFrame([this](const rtc::binary& message, rtc::FrameInfo info) {
            // if (!videoDecoder) return;
            // if (!vidconf->runH264Thread) vidconf->runH264Thread = true;
            // videoDecoder->h264transMode = true;
            // std::vector<uint8_t> imageDataVector(message.size());
            // std::memcpy(imageDataVector.data(), message.data(), message.size());
            // videoDecoder->PushFrame(imageDataVector);
        });

        //videoDecoder->SetDimensions(256, 432); // 50% quality = 320x544, 40% quality = 256x432

        mediaTrackMap.emplace(track->mid(), track);
    }

    void Comms::setupDepthTrack(const std::shared_ptr<rtc::Track>& track) {
        // Set up media pipeline
        const auto mediaHandler = std::make_shared<rtc::H264RtpDepacketizer>(rtc::H264RtpDepacketizer::Separator::LongStartSequence);
        mediaHandler->addToChain(std::make_shared<rtc::RtcpReceivingSession>());
        track->setMediaHandler(mediaHandler);

        // Set up video receiving
        track->onFrame([this](const rtc::binary& message, rtc::FrameInfo info) {
            // if (!depthDecoder) return;
            // if (!vidconf->runH264Thread) vidconf->runH264Thread = true;
            // depthDecoder->h264transMode = true;
            // std::vector<uint8_t> imageDataVector(message.size());
            // std::memcpy(imageDataVector.data(), message.data(), message.size());
            // depthDecoder->PushFrame(imageDataVector);
        });

        //depthDecoder->SetDimensions(512, 512);

        mediaTrackMap.emplace(track->mid(), track);
    }

    void Comms::setupTimingChannel(const std::shared_ptr<rtc::DataChannel>& dc) {
        dc->onClosed([this]() {
            LOG << (localId) << ": "<< "DataChannel pose from " << (remoteId) << " closed";
        });

        dc->onMessage([this](auto data) {
            uint64_t t = std::chrono::steady_clock::now().time_since_epoch().count();
            try {
                if (std::holds_alternative<std::string>(data))
                {
                    auto j = json::parse(std::get<std::string>(data));
                    j["t2"] = t;
                    j["t3"]= std::chrono::steady_clock::now().time_since_epoch().count();
                    SendMsg("time",j.dump());
                }
            } catch (const std::exception& e) {
                std::cout << "Exception caught while receiving incoming bias: " << e.what() << std::endl;
            }
        });
    }

    std::vector<double> s2v(const std::string& str) {
        std::vector<double> ret;
        size_t pos = 0;
        std::string s = str;
        while ((pos = s.find(' ')) != std::string::npos) {
            auto token = s.substr(0,pos);
            if (!token.empty())
                ret.push_back(std::stod(token));
            s.erase(0,pos+1);
        }
        if (!s.empty())
            ret.push_back(std::stod(s));
        
        return ret;
    }

    void Comms::setupControlChannel(const std::shared_ptr<rtc::DataChannel>& dc) {
        dc->onClosed([this]() {
            LOG << (localId) << ": "<< "DataChannel pose from " << (remoteId) << " closed";
        });

        dc->onMessage([this](auto data) {
            if (std::holds_alternative<std::string>(data)) {
                const std::string msg = std::get<std::string>(data);

                LOG << msg;

                auto j = nlohmann::json::parse(std::get<std::string>(data));
                if (m_peerIsImFusion) {
                    j = j["propertyfile"];
                }

                CommandData cd;
                if (j["command"] == "TeleUS") {
                    if (j["payload"] == "PauseTeleop") {
                        cd.cmd = Command::PAUSE_TELEOP;
                    } else if (j["payload"] == "StartCalibration") {
                        //cd.cmd = Command::START_CALIB;
                        if (calState == CalibrationState::Calibrating) 
                            calState = CalibrationState::StopCalibrating;
                        else if (calState == CalibrationState::NotCalibrating)
                            calState = CalibrationState::Calibrating;
                    }
                } 
                else if (j["command"] == "StartSweep") {
                    auto vals = s2v(j["payload"]);
                    if (vals.size() != 6) return;
                    cd.cmd = Command::START_SWEEP;
                    cd.sweepParams = vals;
                }

                if (cd.cmd != Command::NULL_COMMAND)
                    ctrlQ.emplace(cd);

            } else {
                LOG << "Control channel should not receive bytes";
            }
        });
    }

    /* Recall:
    slave_T_flange = slave_T_slave0 * slave_T_master * master_T_hand * ee_T_flange
    */
    void Comms::setupTeleopChannel(const std::shared_ptr<rtc::DataChannel>& dc) {
        dc->onClosed([this]() {
            LOG << (localId) << ": "<< "DataChannel pose from " << (remoteId) << " closed";
        });

        dc->onMessage([this](auto data) {
            try {
                if (std::holds_alternative<std::string>(data))
                {
                    auto j = nlohmann::json::parse(std::get<std::string>(data));
                    if (m_peerIsImFusion) {
                        j = j["propertyfile"];
                    }

                    Isometry3d s_T_m, e_T_f, s_T_s0;
                    {
                        std::scoped_lock lock(poseMutex);
                        s_T_s0 = Isometry3d(slave_T_slave0);
                        s_T_m = Isometry3d(slave_T_master);   
                        e_T_f = Isometry3d(ee_T_flange);
                    }

                    // Transform to local coordinates
                    auto s_R_m = s_T_m.rotation();
                    auto s_R_s0 = s_T_s0.rotation();

                    Vector3d force = slave_T_master_forceScale * s_R_m * vec2eigen<3,1>(s2v(j["force"]));
                    Vector3d torque = {0,0,0}; // Can't get torque from haptic device
                    Vector6d vel = vec2eigen<6,1>(s2v(j["velocity"]));
                    Vector3d linvel = slave_T_master_scale * s_R_s0 * s_R_m * vel.head(3);
                    Vector3d angvel = s_R_s0 * s_R_m * vel.tail(3);

                    Isometry3d m_T_h = Isometry3d(vec2eigen<4,4>(s2v(j["matrix0"]), false)); // Sent in row major!
                    m_T_h = m_T_h.pretranslate((slave_T_master_scale - 1) * m_T_h.translation()); // Scale the transform to the right units (m from mm)

                    RobotData data;
                    data.force_des.head(3) << force;
                    data.force_des.tail(3) << torque;
                    data.vel_des.head(3) << linvel;
                    data.vel_des.tail(3) << angvel;
                    data.pose_des = s_T_s0 * s_T_m * m_T_h * e_T_f;

                    // Place in queue
                    stateQ.emplace(data);
                }
            } catch (const std::exception& e) {
                //std::cout << "Exception caught while handling incoming pose: " << e.what() << std::endl;
            }
        });
    }

    void Comms::setupArucoChannel(const std::shared_ptr<rtc::DataChannel>& dc) {
        dc->onClosed([this]() {
            LOG << (localId) << ": "<< "DataChannel aruco from " << (remoteId) << " closed";
        });

        dc->onMessage([this](auto data) {
            if (calState == CalibrationState::StopCalibrating) computeCalibration();
            if (calState != CalibrationState::Calibrating) return;

            if (!std::holds_alternative<std::string>(data)) return;
            auto j = nlohmann::json::parse(std::get<std::string>(data));
            if (m_peerIsImFusion) {
                j = j["propertyfile"];
            }

            // Sent in row major and mm
            Isometry3d m_T_h = Isometry3d(vec2eigen<4,4>(s2v(j["matrix0"]), false));
            m_T_h = m_T_h.pretranslate((slave_T_master_scale - 1) * m_T_h.translation());
            rgbd_T_marker_mats.push_back(m_T_h);

            // Get the latest slave pose as well
            std::scoped_lock<std::mutex> lock(calibMutex);
            slave_T_flange_mats.push_back(Isometry3d(currentCalibPose));
        });
    }

    void Comms::setupStateChannel(const std::shared_ptr<rtc::DataChannel>& dc) {
        dc->onClosed([this]() {
            LOG << (localId) << ": "<< "DataChannel pose from " << (remoteId) << " closed";
        });

        dc->onMessage([this](auto data) {
            LOG << "Shouldn't be receiving messages on the state channel";
        });
    }

    void Comms::SendMsg(const std::string& channel, const std::string& msg) {
        {
            std::lock_guard<std::mutex> lock(dataChannelMapMutex);
            // Check that the data channel exists
            if (dataChannelMap.find(channel) == dataChannelMap.end())
                return;

            // Send if the channel is open
            if (dataChannelMap.at(channel)->isOpen())
                dataChannelMap.at(channel)->send(msg);
        }
    }

    void Comms::sendState(const RobotData& state)
    {
        sendState(Isometry3d(state.pose_mes), Isometry3d(state.flangePose_mes), state.vel_mes, state.force_mes, state.q_mes, state.angleError);
    }

    /* Recall
    master_T_hand = inv(slave_T_master) * inv(slave_T_slave0) * slave_T_ee * inv(ee_T_flange)
                  = master_T_slave * slave0_T_slave * slave_T_ee * flange_T_ee
    Velocity and force/torque are not affected by the ee_T_flange
    */
    void Comms::sendState(const Isometry3d& poseEE, const Isometry3d& poseFlange, const Vector6d& vel, const Vector6d& wrench, const Vector7d& joints, const double angleError)
    {
        if (!isalive) return;

        Isometry3d m_T_s, f_T_e, s0_T_s;
        {
            std::scoped_lock lock(poseMutex);
            s0_T_s = Isometry3d(slave_T_slave0.inverse());
            m_T_s = Isometry3d(master_T_slave);
            f_T_e = Isometry3d(flange_T_ee);
        }

        double scale = 1/slave_T_master_scale;
        auto m_R_s = m_T_s.rotation();

        Isometry3d pEE = m_T_s * s0_T_s * poseEE;
        Isometry3d pF = m_T_s * s0_T_s * poseFlange;

        if (calState == CalibrationState::Calibrating) {
            std::scoped_lock<std::mutex> lock(calibMutex);
            currentCalibPose = Isometry3d(poseFlange);
        }

        Matrix3d rotEE = pEE.rotation();
        Matrix3d rotF = pF.rotation();
        Vector3d posEE = scale * pEE.translation();
        Vector3d posF = scale * pF.translation();
        Vector3d velLin = scale * m_R_s * vel.head(3);
        Vector3d velRot = m_R_s * vel.tail(3);
        Vector3d force = (1/slave_T_master_forceScale) * m_R_s * wrench.head(3);
        Vector3d torque = (1/slave_T_master_forceScale) * m_R_s * wrench.tail(3);
        sendStateQ.emplace("state", toJson(posEE, rotEE, posF, rotF, velLin, velRot, force, torque, joints, m_peerIsImFusion, angleError));
    }

    void Comms::sendCtrl(const std::string &msg) {
        sendMsgQ.emplace("RtcControl", msg);
    }

    bool Comms::tryGetDesiredState(RobotData& state)
    {
        // Delay if desired
        if (delayMs > 0) {
            if (auto* s = stateQ.peek()) {
                auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                if (t - s->timestamp < delayMs)
                    return false;
            } else return false;
        }

        return stateQ.try_dequeue(state);
    }

    bool Comms::tryGetLastWrench(Vector6d& wrench) {
        return wrenchQ.try_dequeue(wrench);
    }

    bool Comms::tryGetLastAruco(Isometry3d& pose) {
        return mesPoseQ.try_dequeue(pose);
    }

    CommandData Comms::tryGetLastCommand() {
        CommandData cd;
        ctrlQ.try_dequeue(cd);
        return cd;
    }

    void Comms::initCalibration() {
        // Empty the queue in case there are old messages
        Isometry3d tmp;
        while (mesPoseQ.try_dequeue(tmp)) {}
        // Start calibration
        calState = CalibrationState::Calibrating;
    }

    inline Matrix3d Skew(const Vector3d& v) {
        Matrix3d M;
        M << 0,   -v[2], v[1],
             v[2], 0,   -v[0],
            -v[1], v[0], 0;
        return M;
    }

    void Comms::computeCalibration() {
        // From Zhuang et al 1994, DOI 10.1109/70.313105
        // slave_T_flange * flange_T_marker = slave_T_rgbd * rgbd_T_marker
        // => AX = YB
        // A = slave_T_flange_mats, B = rgbd_T_marker_mats
        
        int n = (int)slave_T_flange_mats.size();
        if ((int)rgbd_T_marker_mats.size() != n) {
            LOG << "Size mismatch in list of recorded calibration poses. Could not perform hand-eye calibration";
            return;
        }

        // Create w vector from Eqn 22
        VectorXd C(3*n);
        MatrixXd G(3*n,6);
        for (int i = 0; i < n; i++) {
            // Get quaternions
            Quaterniond aq(slave_T_flange_mats[i].rotation());
            Quaterniond bq(rgbd_T_marker_mats[i].rotation());
            Vector3d a = aq.coeffs().head(3);
            Vector3d b = bq.coeffs().head(3);
            double a0 = aq.coeffs()[0];
            double b0 = bq.coeffs()[0];

            // Append to c vector
            Vector3d c = b - b0/a0 * a;
            for (int j = 0; j < 3; j++)
                C[i*3+j] = c[j];

            // Append to G matrix
            Matrix3d g1 = a0 * Matrix3d::Identity() + Skew(a) + (1/a0) * a*a.transpose();
            Matrix3d g2 = -b0 * Matrix3d::Identity() + Skew(b) - (1/a0) * a*b.transpose();
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    G(i*3+j,k) = g1(j,k);
                    G(i*3+j,k+3) = g2(j,k);
                }
            }
        }

        // Compute w by least squares
        Vector6d w = (G.transpose()*G).inverse() * G.transpose() * C;
//&wsPromise,
        // Compute rotations from w
        double y0 = 1/sqrt(1 + w[3]*w[3] + w[4]*w[4] + w[5]*w[5]);
        Vector3d y,x;
        y << y0*w[3], y0*w[4], y0*w[5];
        x << y0*w[0], y0*w[1], y0*w[2];
        double x0 = 1/sqrt(1 - x[0]*x[0] - x[1]*x[1] - x[2]*x[2]);
        Quaterniond X(x0, x[0], x[1], x[2]);
        Quaterniond Y(y0, y[0], y[1], y[2]);

        // Compute F and D matrices of eqn 31
        VectorXd D(3*n);
        MatrixXd F(3*n,6);
        F.setZero();
        for (int i = 0; i < n; i++) {
            // Get rotation matrices
            Matrix3d fi(slave_T_flange_mats[i].rotation());
            Vector3d di = Y*rgbd_T_marker_mats[i].translation() - slave_T_flange_mats[i].translation();

            for (int row = 0; row < 3; row++) {
                D[i*3+row] = di[row];
                for (int col = 0; col < 3; col++) {
                    F(i*3+row,col) = fi(row,col);
                    if (row == col) F(i*3+row,col+3) = -1;
                }
            }
        }

        // Compute offset by least squares
        Vector6d p = (F.transpose()*F).inverse() * F.transpose() * D;
        Vector3d px(p.head(3));
        Vector3d py(p.tail(3));

        // X = flange_T_marker - don't really care about this
        /*flange_T_marker.setIdentity();
        flange_T_marker.rotate(X);
        flange_T_marker.pretranslate(px);*/

        // Y = slave_T_master
        {
            std::scoped_lock lock(poseMutex);
            //slave_T_masterLast = Isometry3d(slave_T_master);
            // slave_T_master.setIdentity();
            // slave_T_master.rotate(Y);
            // slave_T_master.pretranslate(py);
            // master_T_slave = slave_T_master.inverse();
            Isometry3d sTm = Isometry3d::Identity();
            sTm.rotate(Y);
            sTm.pretranslate(py);
            std::cout << "slave_T_RGBD:\n" << sTm.rotation() << "\nSet the rotation part of slave_T_master and restart. Also set this on the master side." << std::endl;
        }

        // It's important that the slave position and orientation don't suddenly jump
        // Thus, set a flag that warns the next incoming pose to re-bias first
        //calState = CalibrationState::JustCalibrated;

        // Actually we set it manually and restart, so just end it
        calState = CalibrationState::NotCalibrating;
    }

    // Initially had s_T_f0 = s_T_s0 * s_T_m * m_T_h0 * e_T_f = oldPose
    // Now have s_T_f1 = s_T_s1 * s_T_m * m_T_h1 * e_T_f
    // Note, newPose = s_T_s0 * s_T_m * m_T_h1 * e_T_f because the old offset was still used.
    // Need s_T_f1 = s_T_f0 to avoid any jumps in the slave robot
    // Therefore, s_T_s0 * s_T_m * m_T_h0 * e_T_f = s_T_s1 * s_T_m * m_T_h1 * e_T_f
    // => s_T_s1 = (s_T_s0 * s_T_m * m_T_h0 * e_T_f) * inv(s_T_m * m_T_h1 * e_T_f)
    //           = oldPose * inv(inv(s_T_s0) * newPose)
    void Comms::computeOffset(const Isometry3d& newPose, const Isometry3d& oldPose) {
        std::scoped_lock lock(poseMutex);
        // slave_T_slave0 = Isometry3d(oldPose * newPose.inverse() * slave_T_slave0);
        slave_T_slave0 = Isometry3d(oldPose * (slave_T_slave0.inverse() * newPose).inverse());
        initOffset << slave_T_slave0.translation();

        // If we only consider the translation
        //initOffset << oldPose.translation() - newPose.translation();
        //Toff.pretranslate(initOffset);
    }

    void Comms::cleanUp(const bool stopWs){
        // Close and reset the data channels
        {
            std::lock_guard<std::mutex> lock(dataChannelMapMutex);
            // Close and reset the data channels
            for (auto& pair : dataChannelMap) {
                if (pair.second && pair.second->isOpen()) {
                    pair.second->close();
                }
                pair.second.reset();
            }
            if (ws && !ws->isClosed()) {
                ws->close();
            }
        }

        for (const auto& wss : sensorClients)
            wss->close();
        sensorServer->stop();

        if (pc) {
            pc->close();
        }

        LOG << "Comms cleanup completed";
    }
}
