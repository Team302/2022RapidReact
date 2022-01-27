#pragma once
       
enum StateType
{
    INTAKE,
    INTAKE2,
    BALLTRANSFER,
    SHOOTER,
    SHOOTER_HOOD,
    CLIMBER,
    MAX_STATE_TYPES
};


struct StateStruc
{
    int         id;
    StateType   type;
    bool        isDefault;
};
