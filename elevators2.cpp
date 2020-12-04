/*
  Программа после работы выводит время (в секундах) между тем, как человек появился в очереди
  и как он покинул лифт на нужном этаже. В принципе, благодаря созданной структуре people
  можно поддерживать любую информацию о времени для людей.
  В отдельный потоки выделены: каждый лифт, этаж, причем на каждую из двух очередей вверх/вниз свой поток
  и "менеджер" лифтов -- принимает запросы от очередей и раздает команды лифтам
 */

#include <iostream>
#include <thread>
#include <set>
#include <unistd.h>
#include <string>
#include <queue>
#include <deque>
#include <vector>
#include <mutex>

using namespace std;

auto start = chrono::steady_clock::now();

int current_time () {
    auto tt = chrono::steady_clock::now();
    auto res = chrono::duration_cast<chrono::seconds>( tt - start).count() ;
    return  res;
}

struct parameters {
    int N, K, C;
    int T_stage, T_open, T_idle, T_close, T_in, T_out;

    parameters(int N, int K, int C, int T_stage, int T_open, int  T_idle, int T_close, int T_in, int T_out  ) {
        this->N = N;
        this->K = K;
        this->C = C;
        this->T_stage = T_stage;
        this->T_open = T_open;
        this->T_idle = T_idle;
        this->T_close = T_close;
        this->T_in = T_in;
        this->T_out = T_out;
    }
};

struct token { // форма задачи лифта
    int time_in;
    int floor;
    int direction; // направление движение, всюду далее вверх -- положительно, вниз -- отрицаительно

    token ( int T, pair<int, int> P ) {
        time_in = T;
        floor = P.first;
        direction = P.second;
    }

};

bool operator < ( token A, token B ) {
    return A.time_in < B.time_in;
}

set< token > task_manager; // сет для передачи сообщений менеджеру
mutex guard_task_manager;

int control_sum = 0; // кол-во полностью завершенных очередей, нужно для остановки менеджера
mutex guard_control_sum;

struct people {
    int time_in;
    int floor_in;
    int floor_out;
    int time_out;

    people (int AA, int BB, int CC) {
        time_in = AA;
        floor_in = BB;
        floor_out = CC;
        time_out = 0;
    }
};

struct queue_people { // очереди на этажах
    queue<people*> Q;
    mutex* guard_Q;

    queue_people() {
        guard_Q = new mutex;
    }

    bool empty() const {
        guard_Q->lock();
        bool res;
        res = Q.empty();
        guard_Q->unlock();
        return  res;
    }

    void push (people* P ){
        guard_Q->lock();
        Q.push( P );
        guard_Q->unlock();
    }

    people* extract_front () { // запомнить и удалить самого первого человка в очереди
        guard_Q->lock();
        people* P = Q.front();
        Q.pop();
        guard_Q->unlock();
        return  P;
    }

    int get_min_time () {
        return Q.front()->time_in;
    }
};

struct  floor { // структура под этаж
    queue_people* UP;
    queue_people* DOWN;
    mutex* guard_call_manager_up;
    mutex* guard_call_manager_down;

    floor () {
        UP = new queue_people;
        DOWN = new queue_people;
        guard_call_manager_up = new mutex;
        guard_call_manager_down = new mutex;
    }

    static void send_task( pair<int, int> T) { // отправить запрос
        guard_task_manager.lock();
        task_manager.insert( token( current_time(), T)  );
        guard_task_manager.unlock();
    }

    void call_manager ( int n, int direction ) const {
        // мы не должны отправлять запрос, если на этаже уже есть лифт в нужную сторону
        if ( direction > 0 ) {
            guard_call_manager_up->lock();
            if ( !UP->empty() )
                send_task( make_pair(n, 1) ); // лифта нет, передаем запрос
            guard_call_manager_up->unlock();
        }
        else {
            guard_call_manager_down->lock();
            if ( !DOWN->empty() )
                send_task( make_pair(n, -1) );
            guard_call_manager_down->unlock();
        }
    }

};

vector< struct floor*> Floors;

struct elevator { // сам лифт
    int floor;
    string conditional; // состояния: "спит", "ждет" (двери открыты на T_idle), погрузка, едет
    set<people*> people_in; // люди в лифте
    set<int> floor_stop; // необходимо остановиться
    int target_floor; // заказанный изначально этаж
    int target_direction; // заказанное изначально направдение
    int time_start_waiting; // время вхождение в ожидание
    bool flag_open_door; // открыты ли двери
    bool flag_pickup; // для передачи команды от менеджера об открытии дверей и погрузке
    bool flag_process_finish; // если true то выходим из потока


    elevator() {
        floor = 1;
        conditional = "sleeping";
        target_floor = 0;
        target_direction = 0;
        time_start_waiting = 0;
        flag_open_door = false;
        flag_pickup = false;
        flag_process_finish  = false;

    }

    void try_open_doors ( const parameters& P ) {
        if ( flag_open_door )
            return;
        conditional = "opening";
        sleep( P.T_open );
        flag_open_door = true ;
        conditional = "pickuping"; // перешли в погрузку
    }

    void try_close_doors (const string& next_conditional, const parameters& P ) {
        if ( !flag_open_door ) {
            if ( next_conditional == "sleeping" )
                conditional = "sleeping";
            return;
        }
        conditional = "closing";
        sleep( P.T_close );
        flag_open_door = false;
        conditional = next_conditional;
    }

    void output_one_person (people* P, const parameters& Par) {
        try_open_doors( Par );
        people_in.erase(P);
        sleep( Par.T_out );
        P->time_out = current_time();
    }

    void output ( const parameters& P ) {
        queue<people*> Q_out;
        for ( auto it : people_in )
            if ( it->floor_out == floor )
                Q_out.push( it );
        while ( ! Q_out.empty() ) {
            output_one_person( Q_out.front(), P );
            Q_out.pop();
        }
        floor_stop.erase( floor );
    }

    void input_one_person ( people* P, const parameters& Par ) {
        try_open_doors(Par);
        sleep( Par.T_in );
        people_in.insert( P );
        floor_stop.insert( P->floor_out );
    }

    void move_up (const parameters& P ) {
        try_close_doors("moving_up", P);
        conditional = "moving_up";
        ++floor;
        sleep( P.T_stage);
    }

    void move_down ( const parameters& P) {
        try_close_doors("moving_down", P);
        conditional = "moving_down";
        --floor;
        sleep( P.T_stage );
    }

    void continue_move ( const parameters& P ) {
        // продолжить/начать движение, если везем людей или не достигли заказанного этажа
        if ( floor_stop.empty() ) {
            if ( target_floor > floor ) {
                move_up( P );
                return;
            }
            else {
                move_down( P );
                return;
            }
        }
        if ( *(floor_stop.begin() ) > floor )
            move_up( P );
        else
            move_down( P );
    }

    void pick_up ( struct floor* F, int direction, const parameters& Par ) { // процедура разгрузки

        if ( direction > 0 )  // ставим mutex на соответствующие направление против вызова второго лифта
            F->guard_call_manager_up->lock();
        else
            F->guard_call_manager_down->lock();

        if ( floor == target_floor )
            target_floor = 0;

        output( Par ); // выпускаем всех кого надо

        while ( people_in.size() < Par.C ) { //запускаем всех с нужным направлением, пока не переполнимся
            if ( direction > 0 ) {
                if ( F->UP->empty() )
                    break;
                people* P = F->UP->extract_front();
                input_one_person(P, Par);
            }
            else {
                if ( F->DOWN->empty() )
                    break;
                people* P = F->DOWN->extract_front();
                input_one_person(P, Par);
            }
        }

        if ( direction > 0 ) // погрузка окончена, отдаем mutex
            F->guard_call_manager_up->unlock();
        else
            F->guard_call_manager_down->unlock();

        if (  target_floor > 0  || !floor_stop.empty() ) //не пусти или этот этаж только промежуточный
            continue_move( Par);
        else  { // достигли заказанного этажа и никого нет
            conditional = "waiting";
            time_start_waiting = current_time();
        }
    }
};

vector<elevator*> El;

void thread_elevator ( int n, const parameters& P ) {
    while ( true ) {
        // следим за командами от менеджера и исполняем их или засыпаем
        if ( El[n]->conditional == "sleeping" ) {
            if ( El[n]->flag_pickup ) {
                El[n]->flag_pickup = false;
                El[n]->pick_up(Floors[El[n]->floor], El[n]->target_direction, P);
                continue;
            }
            if ( El[n]->target_floor > 0 ) {
                if ( El[n]->target_floor == El[n]->floor ) {
                    El[n]->pick_up( Floors[El[n]->floor], El[n]->target_floor, P);
                    continue;
                }
                El[n]->continue_move( P );
                continue;
            }
            if (El[n]->flag_process_finish )
                return;
            continue;
        }
        if ( El[n]->conditional == "waiting" ) {
            if ( El[n]->flag_pickup ) {
                El[n]->flag_pickup = false;
                El[n]->pick_up( Floors[El[n]->floor] , El[n]->target_direction, P );
                continue;
            }
            if ( El[n]->time_start_waiting + P.T_idle <= current_time() )
                El[n]->try_close_doors("sleeping", P );
            continue;
        }
        if ( El[n]->conditional == "moving_up" || El[n]->conditional == "moving_down"  ) {
            if ( El[n]->flag_pickup || El[n]->floor_stop.find(El[n]->floor) != El[n]->floor_stop.end() ) {
                if (El[n]->flag_pickup )
                    El[n]->flag_pickup = false;
                El[n]->pick_up( Floors[El[n]->floor], El[n]->target_direction, P );
                continue;
            }
            if ( El[n]->floor == El[n]->target_floor ) {
                El[n]->pick_up( Floors[El[n]->floor], El[n]->target_direction, P );
                continue;
            }
            El[n]->continue_move( P );
        }
    }
}

void thread_floor_UP (int n, queue_people& Q) { // поток очереди наверх
    while ( !Q.empty() ) { // не все люди уехали
        if ( Q.get_min_time() <= current_time() ) { //надо ли выхывать лифт
            people* P = Q.extract_front();
            Floors[n]->UP->push(P);
            Floors[n]->call_manager( n, 1);
        }
        else
            continue;
    }
    while ( !Floors[n]->UP->empty() ) {
        if ( !Floors[n]->UP->empty() )
            Floors[n]->call_manager(n, 1);
    }
    guard_control_sum.lock();
    ++control_sum; // завершаем, увеличиваем сумму
    guard_control_sum.unlock();
}

void thread_floor_DOWN (int n, queue_people& Q) { // аналогично, только вних
    while ( !Q.empty() ) {
        if ( Q.get_min_time() <= current_time() ) {
            people* P = Q.extract_front();
            Floors[n]->DOWN->push(P);
            Floors[n]->call_manager( n, -1);
        }
        else
            continue;
    }
    while ( !Floors[n]->DOWN->empty() ) {
        if ( !Floors[n]->DOWN->empty() )
            Floors[n]->call_manager(n, -1);
    }
    guard_control_sum.lock();
    ++control_sum;
    guard_control_sum.unlock();
}

int search_waiting ( pair<int, int> P, const parameters& Par ) { // ищем подходящий ждущий лифт
    for ( int i = 0; i < Par.K; ++i )
        if ( El[i]->conditional == "waiting" && El[i]->floor == P.first )
            return i;
    return -1;
}

int search_moving ( pair<int, int> P, const parameters& Par ) {  // если лифт проходит как раз в нужном направлении
    for ( int i = 0; i < Par.K; ++i )
        if (El[i]->floor == P.first && El[i]->target_direction == P.second )
            return i;
    return -1;
}

int search_sleeping ( pair<int, int> P, const parameters& Par ) { // ищем среди спящих
    int res = -1;
    int min_delta = Par.N +2 ;
    for ( int i = 0; i < Par.K; ++i )
        if ( El[i]->conditional == "sleeping" ) {
            if ( abs(El[i]->floor - P.first) < min_delta ) {
                min_delta = abs(El[i]->floor - P.first);
                res = i;
            }
        }
    return res;
}

void delete_task (token task ) {
    guard_task_manager.lock();
    task_manager.erase( task);
    guard_task_manager.unlock();
}

void give_task ( int n, pair<int, int> task, token T) {
    if ( El[n]->conditional == "moving_up" || El[n]->conditional == "moving_down" ||
         El[n]->conditional == "waiting ") {
        El[n]->flag_pickup = true;
        delete_task(  T );
        return;
    }
    El[n]->target_floor = task.first;
    El[n]->target_direction =task.second;
    if ( El[n]->floor == task.first )
        El[n]->flag_pickup = true;
    sleep(static_cast<unsigned int>(0.001));
    delete_task( T );
}

void thread_manager ( const parameters& Par) { // поток менеджера
    while ( true ) {
        guard_control_sum.lock();
        guard_task_manager.lock();
        if ( control_sum == 2 * Par.N && task_manager.empty()) { //все люди разъехались
            guard_control_sum.unlock();
            guard_task_manager.unlock();
            for ( int i = 0; i < Par.K; ++i )  // приказываем лифтам завершится
                El[i]->flag_process_finish = true;
            return; //кончиились задания
        }
        else {
            guard_control_sum.unlock();
            vector< token > current_task;
            current_task.reserve(task_manager.size());
            for ( auto  it : task_manager ) // снимаем все текущие задачи и обрабатываем
                current_task.push_back( it);
            guard_task_manager.unlock();
            if ( current_task.empty() )
                continue;
            else {
                for ( auto to : current_task ) {
                    pair<int, int> task = make_pair( to.floor, to.direction);
                    int flag = search_waiting( task, Par);
                    if ( flag != -1 ) {
                        give_task( flag, task, to );
                        continue;
                    }
                    flag = search_moving( task, Par);
                    if ( flag != -1 ) {
                        give_task( flag, task, to);
                        continue;
                    }
                    flag = search_sleeping( task, Par );
                    if ( flag != -1 ) {
                        give_task( flag,  task, to );
                        continue;
                    }
                }
            }
        }
    }
}

int get_time ( const string & S) { // перевод времени
    int res = 0;
    res += (S[0] -'0') * 10 * 60 * 60;
    res += (S[1] -'0') * 60 * 60;
    res += (S[3] -'0') * 10 * 60;
    res += (S[4] -'0') * 60;
    res += (S[6] -'0') * 10;
    res += (S[7] -'0');
    return res;
}

int main() {
    int N, K, C;
    int T_stage, T_open, T_idle, T_close, T_in, T_out;

    cin>> N >> K >> C;
    cin>>T_stage >> T_open >> T_idle >> T_close >> T_in >> T_out;

    const parameters Par (N, K, C, T_stage, T_open, T_idle, T_close, T_in, T_out );


    for ( int i = 0; i <= N; ++i)
        Floors.push_back( new floor );

    El.resize ( K );

    for ( int i = 0 ; i < K; ++i )
        El[i] = new elevator;

    int kol; // кол-во людей
    cin>> kol;

    vector< people* > all_people ;
    vector < pair<queue_people, queue_people> > Q ( N+1 );

    int time_zero; // самое минимальное время прихода, относительно его и будем считать

    for ( int i = 0 ; i < kol; ++i ) {
        string  S;
        int f1, f2;
        cin >> S;
        cin >> f1 >> f2;
        if ( i == 0 )
            time_zero = get_time( S );
        int tt = get_time( S ) - time_zero;
        auto* P = new people( tt, f1, f2);
        if ( f2 > f1 )
            Q[f1].first.push(P);
        else
            Q[f1].second.push(P);
        all_people.push_back( P );
    }
    start = chrono::steady_clock::now();

    vector< pair<thread, thread> > th_floors ( N + 1 );
    for ( int i = 1; i <= N; ++i ) {
        th_floors[i].first = thread ( thread_floor_UP, i, ref(Q[i].first));
        th_floors[i].second = thread ( thread_floor_DOWN, i, ref(Q[i].second));
    }

    vector<thread> th_elevator;
    th_elevator.reserve(K);
    for ( int i = 0; i < K; ++i)
        th_elevator.emplace_back( thread_elevator, i , Par);

    thread th_manager = thread( thread_manager, Par );


    for ( int i = 1 ; i <= N; ++i ) {
        th_floors[i].first.join();
        th_floors[i].second.join();
    }

    th_manager.join();

    for ( int i = 0; i < K; ++i)
        th_elevator[i].join();



    for ( auto it : all_people )
        cout<< it->time_out - it->time_in <<' ';

}
