#pragma once
       
enum StateType
{
    INTAKE,
    BALLTRANSER,
    ARM,
    BALLRELEASE,
    SHOOTER,
    MAX_STATE_TYPES
};


struct StateStruc
{
    int         id;
    StateType   type;
    bool        isDefault;
};