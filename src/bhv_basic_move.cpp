// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"

#include "strategy.h"

#include "bhv_basic_tackle.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "neck_offensive_intercept_neck.h"
#include "chain_action/field_analyzer.h"
#include <iostream>
using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicMove::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove" );

    //-----------------------------------------------
    // tackle
    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {
        return true;
    }

    const WorldModel & wm = agent->world();
    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    if ( ! wm.existKickableTeammate()
         && ( self_min <= 3
              || ( self_min <= mate_min
                   && self_min < opp_min + 3 )
              )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );

        return true;
    }

    if(opp_min <= std::min(self_min, mate_min))
    {
        if(Bhv_Block().execute(agent))
            return true;
    }

    const Vector2D target_point = Strategy::i().getPosition( wm.self().unum() );
    const double dash_power = Strategy::get_normal_dash_power( wm );

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                  target_point.x, target_point.y,
                  dist_thr );

    agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
    agent->debugClient().setTarget( target_point );
    agent->debugClient().addCircle( target_point, dist_thr );

    if ( ! Body_GoToPoint( target_point, dist_thr, dash_power
                           ).execute( agent ) )
    {
        Body_TurnToBall().execute( agent );
    }

    if ( wm.existKickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan() );
    }

    return true;
}

bool Bhv_Block::execute(PlayerAgent * agent)
{
    const WorldModel & wm = agent->world();
    updateBlockCycle(wm);
    updateBlockerUnum();
    std::cout<<"block,"<<wm.time().cycle()<<","<<wm.self().unum()<<","<<blockerUnum<<std::endl;
    if (blockerUnum == wm.self().unum())
    {
        Vector2D targetPos = blockPos[wm.self().unum()];
        if(!Body_GoToPoint2010(targetPos, 0.1, 100, 1.2, 1, false, 20).execute(agent))
        {
            Body_TurnToPoint(targetPos).execute(agent);
        }
        agent->debugClient().addCircle(targetPos, 1);
        agent->setNeckAction( new Neck_TurnToBallOrScan() );
        return true;
    }
    return false;
}

void Bhv_Block::updateBlockerUnum()
{
    int blockC = INT_MAX;
    for(int unum = 2; unum <= 11; unum++){
        if(blockCycle[unum] < blockC)
        {
            blockC = blockCycle[unum];
            blockerUnum = unum;
        }
    }
}

void Bhv_Block::updateBlockCycle(const WorldModel &wm)
{
    int oppMin = wm.interceptTable()->opponentReachCycle();
    Vector2D startDribble = wm.ball().inertiaPoint(oppMin);
    AngleDeg dribbleAngle = (ServerParam::i().ourTeamGoalPos() - startDribble).th();
    double dribbleSpeed = 0.8;
    Vector2D dribbleVel = Vector2D::polar2vector(dribbleSpeed, dribbleAngle);

    for(int unum = 1; unum <= 11; unum++)
    {
        Vector2D kickPos = startDribble;
        blockCycle[unum] = INT_MAX;
        const AbstractPlayerObject * tm = wm.ourPlayer(unum);
        if(tm == nullptr || tm->unum() != unum)
            continue;
        dlog.addText(Logger::BLOCK, "TM %d >>>>>>>>>>>>>>>>>>>>>>", unum);
        for(int cycle = oppMin + 1; cycle < oppMin + 30; cycle++)
        {
            kickPos += dribbleVel;
            int reachCycle = getBlockCycle(tm, kickPos, cycle);
            dlog.addText(Logger::BLOCK, "cycle:%d, pos:%.1f,%.1f, reach:%d", cycle, kickPos, reachCycle);
            if(reachCycle <= cycle)
            {
                dlog.addLine(Logger::BLOCK, tm->pos(), kickPos,255,0,0);
                blockCycle[unum] = cycle;
                blockPos[unum] = kickPos;
                break;
            }
        }
    }
}

int Bhv_Block::getBlockCycle(const AbstractPlayerObject *tm, Vector2D dribblePos, int cycle)
{
    Vector2D tmPos = tm->inertiaPoint(cycle);
    double kickableArea = tm->playerTypePtr()->kickableArea();
    if(tm->pos().dist(dribblePos) < kickableArea)
        return 0;
    if(tmPos.dist(dribblePos) < kickableArea)
        return 0;



    double dist = tmPos.dist(dribblePos) - kickableArea;
    int dashCycle = tm->playerTypePtr()->cyclesToReachDistance(dist);
    int turnCycle = 0;

    double diffAngle = ((dribblePos - tmPos).th() - tm->body()).abs();
    double speed = tm->vel().r();
    while(diffAngle > 15)
    {
        diffAngle -= tm->playerTypePtr()->effectiveTurn( ServerParam::i().maxMoment(), speed );
        speed *= tm->playerTypePtr()->playerDecay();
        turnCycle++;
    }
    return dashCycle + turnCycle;
    int reachCycle = FieldAnalyzer::predict_player_reach_cycle(tm,
                                                              dribblePos,
                                                              kickableArea,
                                                              0,
                                                              0,
                                                              0,
                                                              0,
                                                              false);
    return reachCycle;
}
