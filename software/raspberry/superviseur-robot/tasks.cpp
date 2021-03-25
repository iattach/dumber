/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotMsgLost, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobotWD, NULL, 0, S_FIFO)) {  //fonction 11 create sempahore
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_errSocket, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_errSocketRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_restartServer, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobotWD, "th_startRobotWD", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_detectComLostMonitor, "th_detectComLostMonitor", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_detectComLostRobot, "th_detectComLostRobot", 0,PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_levelBat, "th_levelBat", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobotWD, (void(*)(void*)) & Tasks::StartRobotTaskWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_detectComLostMonitor, (void(*)(void*)) & Tasks::DetectComLostMonitor, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_detectComLostRobot, (void(*)(void*)) & Tasks::DetectComLostRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_levelBat, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            rt_sem_v(&sem_errSocket);
            //rt_sem_p(&sem_serverOk, TM_INFINITE);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
            rt_sem_v(&sem_errSocketRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        }else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            rt_mutex_release(&mutex_watchdog);
            rt_sem_v(&sem_startRobot);

        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}


/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;


        cout << "Start answer: " << msgSend->ToString() << endl << flush;

        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
                        
            rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
            robotMsgLost = 0;
            rt_mutex_release(&mutex_robotMsgLost);
        }else{
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted=1;
            rt_mutex_release(&mutex_robotStarted);
                        
            rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
            robotMsgLost++;
            rt_mutex_release(&mutex_robotMsgLost);
            //rt_sem_v(&sem_errRobot);
        }
    }
        
    
}

/**
 * @brief Thread starting the communication with the robot with watchdog.
 */
void Tasks::StartRobotTaskWD(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobotWD, TM_INFINITE);
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());//StartWithoutWD
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Start answer: " << msgSend->ToString() << endl << flush;
        
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            
            robotStarted = 1;
            
           // int err;
            
           /* if (err = rt_task_start(&th_startRobotWD, (void(*)(void*)) & Tasks::ReloadWatchdog, this)) {
                cerr << "Error task start: " << strerror(-err) << endl << flush;
                exit(EXIT_FAILURE);
            }*/
           
            rt_mutex_release(&mutex_robotStarted);
        }
        
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be sended to Monitor and be deleted by sendToMon

    }
}

void Tasks::ReloadWatchdog(void *arg) {
    
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    
    Message * msgSend;
    cout << "Reload watchdog (";
    msgSend = robot.Write(robot.ReloadWD());
    cout << msgSend->GetID();
    cout << ")" << endl;
    
    WriteInQueue(&q_messageToMon, msgSend);
 
}
/**
 * @brief Thread handling control of the robot.
 * 
 * Fonction : déplacer robot
 * 
 * tant que périodiquement (100 ms) 
 *  si robot démarré est vrai    
 *      lire mouvement    
 *      envoyer ordre de mouvement au robot
 *  fin si
 * fin tant que
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int lost =0;
    int cpMove;
    Message* msgSend;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);//tant que périodiquement (100 ms) 

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1  ) {//si robot démarré est vrai
            
            
                cout << "Periodic movement update";
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                cpMove = move;//lire mouvement   
                rt_mutex_release(&mutex_move);
                
            if(1){ 
                cout << " move: " << cpMove;

                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(new Message((MessageID)cpMove));//envoyer ordre de mouvement au robot
                rt_mutex_release(&mutex_robot);
                cout << endl << flush;
           
            }
            if (msgSend->GetID() == MESSAGE_ANSWER_ACK ) {    
                rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
                robotMsgLost = 0;
                rt_mutex_release(&mutex_robotMsgLost);
                lost = 1 ;
            }else{        
                lost = 0;
                rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
                robotMsgLost++;
                rt_sem_v(&sem_errSocketRobot);
                rt_mutex_release(&mutex_robotMsgLost);
                //rt_sem_v(&sem_errRobot);
            }
            
        }//fin si
        
    }
}
/**
 * @brief Thread handling battery.
 * 

Fonction : récupérer le niveau de la batterie

tant que péridodiquement (500ms)
 poster messageToMon levelBat
fin tant que s
 * 
 */
void Tasks::BatteryTask(void *arg) {
    int rs;
    int cpMove;
    int lost = 0;
    Message* msgSend;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);//tant que périodiquement (100 ms) 

    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {//si robot démarré est vrai  
            
           if (1){ 
            cout << endl << "Level battery update";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);

            msgSend = robot.Write(robot.GetBattery());//envoyer ordre de mouvement au robot
            rt_mutex_release(&mutex_robot);


            //CheckRobotMessage(levelBat);

            WriteInQueue(&q_messageToMon,msgSend);
           }
           if (msgSend->GetID() == MESSAGE_ROBOT_BATTERY_LEVEL  ) {    
                rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
                robotMsgLost = 0;
                rt_mutex_release(&mutex_robotMsgLost);
                lost = 0;
            }else{
               lost = 1;
                rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
                robotMsgLost++;
                rt_sem_v(&sem_errSocketRobot);
                rt_mutex_release(&mutex_robotMsgLost);
            //rt_sem_v(&sem_errRobot);
            }
            
        }//fin si
    }
}



/**********************************************************************/
/* Queue services                                                     */
/**********************************************************************/
/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**
 * @brief Thread detecting communication lost with robot.
 */
void Tasks::DetectComLostMonitor(void *arg){
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        rt_sem_p(&sem_errSocket, TM_INFINITE);
        
        //fonction 5
        cout << "Communication between monitor and supervisor lost" << endl;
        
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        rt_mutex_release(&mutex_monitor);        
        //fonction 6
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        int rs= robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
       
        if (rs){
            Message * msgSend;
            //fonction 6
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = MESSAGE_ROBOT_STOP;
            rt_mutex_release(&mutex_move);
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);
            
            cout << "Stop robot " << endl << flush;
               
            rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
            robotMsgLost = 0;
            rt_mutex_release(&mutex_robotMsgLost);
            
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);     
           
        }
        
        /**************************************************************************************/
        /* The task server starts here                                                        */
        /**************************************************************************************/
     

        cout << "Stop server "<< endl;
        
        rt_sem_broadcast(&sem_serverOk);
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera.Close();
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted=0;
        rt_mutex_release(&mutex_cameraStarted);
        
        cout << "Stop camera "<< endl;
        

    }
}

void Tasks::DetectComLostRobot(void *arg){
    int cpt =0;
    Message * msgSend;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        rt_sem_p(&sem_errSocketRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
         cpt = robotMsgLost;
        rt_mutex_release(&mutex_robotMsgLost);
        //fonction 5
        cout << "Communication between robot and supervisor lost" << endl;
        
        if(cpt==3){
            
            msgSend=new Message(MESSAGE_ANSWER_ROBOT_TIMEOUT);
            WriteInQueue(&q_messageToMon,msgSend);
                  
            //fonction 6
            
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted=0;
            cout << "state of robot : "<<robotStarted<<endl;
            rt_mutex_release(&mutex_robotStarted);
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);
            
            /*
            if (rs){
                Message * msgSend;
                //fonction 6
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = MESSAGE_ROBOT_STOP;
                rt_mutex_release(&mutex_move);

                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close();
                rt_mutex_release(&mutex_robot);

                cout << "Stop robot " << endl << flush;

                rt_mutex_acquire(&mutex_robotMsgLost, TM_INFINITE);
                robotMsgLost = 0;
                rt_mutex_release(&mutex_robotMsgLost);

                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);     

            }*/
        
            /**************************************************************************************/
            /* The task server starts here                                                        */
            /**************************************************************************************/

            /*
            cout << "Stop server "<< endl;

            rt_sem_broadcast(&sem_serverOk);
            rt_sem_p(&sem_serverOk, TM_INFINITE);

            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera.Close();
            rt_mutex_release(&mutex_camera);

            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted=0;
            rt_mutex_release(&mutex_cameraStarted);

            cout << "Stop camera "<< endl;*/
        }
        

    }
}
