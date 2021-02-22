#pragma once

#include "psmove/psmove.h"
#include "psmove/psmove_tracker.h"
#include "psmove/psmove_fusion.h"

#include "raylib.h"
#include "opencv2/videoio.hpp"


#include <unordered_map>
#include <optional>
#include <vector>
#include <string>
#include <memory>

template <class EnumClass, class Context>
class ConfigureState
{
public:
    using ResultType = std::optional<EnumClass>;

    ConfigureState( const std::string& name, Context& context) :
        _name(name),
        _context(context) {}
    virtual ~ConfigureState() {};

    const std::string& name() const { return _name; }

    virtual ResultType onStateEnter() { return {}; };
    virtual ResultType onStateUpdate() { return {}; };
    virtual ResultType onStateExit() { return {}; };
protected:
    std::string _name;
    Context& _context;
};

template <class EnumClass, class Context>
class ConfigureMachine
{
public:
    using StateType = EnumClass;
    using ResultType = std::optional<EnumClass>;

    ConfigureMachine(Context& context) :
        _context(context) {}

    template <class State>
    void addState(StateType state, const std::string& name)
    {
        auto ret = _states.emplace(state, std::make_unique<State>(name, _context));
        if (ret.second)
        {
            _stateNames.push_back(name);
            if (_firstState)
            {
                _currentState = state;
                _firstState = false;
            }
        }
        else
        {
            return;
        }
    }

    void changeState(StateType state)
    {
        auto currIt = _states.find(_currentState);
        assert(currIt != _states.end());
        currIt->second->onStateExit();
        auto iter = _states.find(state);
        assert(iter != _states.end());
        _currentState = state;
        ResultType transition = iter->second->onStateEnter();
        if (transition)
        {
            changeState(*transition);
        }
    }

    StateType currentState() const
    {
        return _currentState;
    }

    std::optional<StateType> stateFromName(const std::string& name)
    {
        for (const auto& pair: _states)
        {
            if (pair.second->name() == name)
            {
                return pair.first;
            }
        }

        return {};
    }

    void tick()
    {
        auto currIt = _states.find(_currentState);
        assert(currIt != _states.end());
        ResultType transition = currIt->second->onStateUpdate();
        if (transition)
        {
            changeState(*transition);
        }
    }

    const std::vector<std::string>& stateNames() { return _stateNames; };

private:
    bool _firstState{ true };
    StateType _currentState;
    Context& _context;
    std::vector<std::string> _stateNames;
    std::unordered_map<StateType, std::unique_ptr<ConfigureState<StateType, Context>>> _states;
};



enum class States {
    Main,
    ConfigureCamera,
    ConfigureController,
    ConfigureTracker,
    TestFusion,
};

struct ConfigureContext;
using Configure = ConfigureState<States, ConfigureContext>;
using SM = ConfigureMachine<States, ConfigureContext>;


struct ConfigureContext
{
    ConfigureContext();
    ~ConfigureContext();
    bool shouldExit{ false };
    int screenWidth{ 1920 };
    int screenHeight{ 1080 };
    SM sm;
    Model psMoveModel{};
};


class ConfigureMain : public Configure
{
public:
    ConfigureMain(const std::string& name, ConfigureContext& context) : Configure(name, context) {}
    virtual ResultType onStateEnter() override;
    virtual ResultType onStateUpdate() override;
    virtual ResultType onStateExit() override;
};

class ConfigureCamera : public Configure
{
public:
    ConfigureCamera(const std::string& name, ConfigureContext& context) : Configure(name, context) {}
    virtual ResultType onStateEnter() override;
    virtual ResultType onStateUpdate() override;
    virtual ResultType onStateExit() override;
private:
    cv::Mat frame;
    cv::Mat threshHSV;
    cv::Mat thresh;
    Texture2D videoTex{};
    Texture2D threshTex{};
    std::vector<int> minHSV{0, 0, 0};
    std::vector<int> maxHSV{179, 255, 255};
    std::unique_ptr<cv::VideoCapture> capture;
    std::vector<PSMove*> controllers;
    size_t currentController{ 0 };
};

class ConfigureController : public Configure
{
public:
    ConfigureController(const std::string& name, ConfigureContext& context) : Configure(name, context) {}
    virtual ResultType onStateEnter() override;
    virtual ResultType onStateUpdate() override;
    virtual ResultType onStateExit() override;

 private:
     void magnetormeterCalibrationUpdate();
     bool isMoveStableAndAlignedWithGravity();
private:
    struct Controller
    {
        PSMove* move{};
        int R{};
        int G{};
        int B{};
    };
    std::vector<Controller> controllers;
    Camera3D camera {0};
};

class ConfigureTracker : public Configure
{
public:
    ConfigureTracker(const std::string& name, ConfigureContext& context) : Configure(name, context) {}
    virtual ResultType onStateEnter() override;
    virtual ResultType onStateUpdate() override;
    virtual ResultType onStateExit() override;
private:
    cv::Mat frame;
    Texture2D videoTex{};
    std::vector<PSMove*> controllers;
    PSMoveTrackerSettings settings;
    std::unique_ptr<PSMoveTracker> tracker;
};

class TestFusion : public Configure
{
public:
    TestFusion(const std::string& name, ConfigureContext& context) : Configure(name, context) {}
    virtual ResultType onStateEnter() override;
    virtual ResultType onStateUpdate() override;
    virtual ResultType onStateExit() override;
private:
    std::vector<PSMove*> controllers;
    PSMoveTrackerSettings settings;
    std::unique_ptr<PSMoveTracker> tracker;
    std::unique_ptr<PSMoveFusion> fusion;
    Camera3D camera{ 0 };
};

