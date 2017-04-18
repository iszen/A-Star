
#include <iostream>
#include <math.h>
#include <ctime>
#include <string>
#include <iomanip>
#include <queue>

using namespace std;


static int horizontal_neighbors[] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static int vertical_neighbors[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

const int map_width = 30;
const int map_height = 30;
static int map_matrix[map_width][map_height];

static int consumed_fields[map_width][map_height];
static int open_fields[map_width][map_height];

//stores the direction from where we got to the field for each field
static int parent_directions[map_width][map_height];

class field
{
	int x;
	int y;
	int distance_from_start;
	int distance_to_goal;

public:
	//constructor
	field(int x_position, int y_position, int distance, int distance_estimate)
	{
		x = x_position; y = y_position; distance_from_start = distance; distance_to_goal = distance_estimate;
	}

	//getters
	int getxPos() const { return x; }
	int getyPos() const { return y; }
	int getStartDistance() const { return distance_from_start; }
	int getGoalDistance() const { return distance_to_goal; }

	void setGoalDistance(const int & x_goal, const int & y_goal)
	{
		distance_to_goal = distance_from_start + estimate(x_goal, y_goal) * 10;
	}

	void nextStep(const int & direction)
	{
		distance_from_start += direction % 2 == 0 ? 10 : 14;
	}

	const int & estimate(const int & x_goal, const int & y_goal) const
	{
		static int x_distance, y_distance, distance;
		x_distance = x_goal - x;
		y_distance = y_goal - y;
		distance = static_cast<int>(sqrt(x_distance*x_distance + y_distance*y_distance));
		return(distance);
	}
};

bool operator<(const field & field_1, const field & field_2)
{
	//compare two fields to decide wich is a better choice
	return field_1.getGoalDistance() > field_2.getGoalDistance();
}

string findPath(const int & x_Start, const int & y_Start,
	const int & x_Goal, const int & y_Goal)
{
	static priority_queue<field> fields_priority_queue[2];
	static int index;
	static field* field_1;
	static field* field_2;
	static int direction_index, neighbors_index, x_index_map, y_index_map, next_x_direction, next_y_direction;
	static char path_char;
	index = 0;

	//set all fields to 0
	//initialize the open map
	for (y_index_map = 0; y_index_map < map_height; y_index_map++)
	{
		for (x_index_map = 0; x_index_map < map_width; x_index_map++)
		{
			consumed_fields[x_index_map][y_index_map] = 0;
			open_fields[x_index_map][y_index_map] = 0;
		}
	}

	//initialize start and goal fields and add to open list(matrix)
	field_1 = new field(x_Start, y_Start, 0, 0);
	field_1->setGoalDistance(x_Goal, y_Goal);
	fields_priority_queue[index].push(*field_1);
	open_fields[x_index_map][y_index_map] = field_1->getGoalDistance();

	//while there are still nodes in the open list(matrix)
	while (!fields_priority_queue[index].empty())
	{
		field_1 = new field(fields_priority_queue[index].top().getxPos(), fields_priority_queue[index].top().getyPos(),
			fields_priority_queue[index].top().getStartDistance(), fields_priority_queue[index].top().getGoalDistance());

		x_index_map = field_1->getxPos();
		y_index_map = field_1->getyPos();

		fields_priority_queue[index].pop();

		open_fields[x_index_map][y_index_map] = 0;

		consumed_fields[x_index_map][y_index_map] = 1;

		//if goal is reached
		if (x_index_map == x_Goal && y_index_map == y_Goal)
		{
			//get the path as string
			string path = "";
			while (!(x_index_map == x_Start && y_index_map == y_Start))
			{
				neighbors_index = parent_directions[x_index_map][y_index_map];
				path_char = '0' + (neighbors_index + 4) % 8;
				path = path_char + path;
				x_index_map += horizontal_neighbors[neighbors_index];
				y_index_map += vertical_neighbors[neighbors_index];
			}

			//cleanup :)
			delete field_1;
			while (!fields_priority_queue[index].empty()) fields_priority_queue[index].pop();
			return path;
		}

		// try out all possible directions
		for (direction_index = 0; direction_index < 8; direction_index++)
		{
			next_x_direction = x_index_map + horizontal_neighbors[direction_index];
			next_y_direction = y_index_map + vertical_neighbors[direction_index];

			if (!(next_x_direction<0 || next_x_direction>map_width - 1 || next_y_direction<0 || next_y_direction>map_height - 1 || map_matrix[next_x_direction][next_y_direction] == 1
				|| consumed_fields[next_x_direction][next_y_direction] == 1))
			{
				field_2 = new field(next_x_direction, next_y_direction, field_1->getStartDistance(),
					field_1->getGoalDistance());
				field_2->nextStep(direction_index);
				field_2->setGoalDistance(x_Goal, y_Goal);

				if (open_fields[next_x_direction][next_y_direction] == 0)
				{
					open_fields[next_x_direction][next_y_direction] = field_2->getGoalDistance();
					fields_priority_queue[index].push(*field_2);
					parent_directions[next_x_direction][next_y_direction] = (direction_index + 4) % 8;
				}
				else if (open_fields[next_x_direction][next_y_direction] > field_2->getGoalDistance())
				{
					open_fields[next_x_direction][next_y_direction] = field_2->getGoalDistance();
					parent_directions[next_x_direction][next_y_direction] = (direction_index + 4) % 8;

					while (!(fields_priority_queue[index].top().getxPos() == next_x_direction &&
						fields_priority_queue[index].top().getyPos() == next_y_direction))
					{
						fields_priority_queue[1 - index].push(fields_priority_queue[index].top());
						fields_priority_queue[index].pop();
					}
					fields_priority_queue[index].pop();

					if (fields_priority_queue[index].size() > fields_priority_queue[1 - index].size()) index = 1 - index;
					while (!fields_priority_queue[index].empty())
					{
						fields_priority_queue[1 - index].push(fields_priority_queue[index].top());
						fields_priority_queue[index].pop();
					}
					index = 1 - index;
					fields_priority_queue[index].push(*field_2);
				}
				else delete field_2;
			}
		}
		delete field_1;
	}
	//there is no way
	return ""; 
}

int main()
{
	srand(time(NULL));

	for (int y = 0; y < map_height; y++)
	{
		for (int x = 0; x < map_width; x++) map_matrix[x][y] = 0;
	}

	for (int x = map_width / 8; x < map_width * 7 / 8; x++)
	{
		map_matrix[x][map_height / 2] = 1;
	}
	for (int y = map_height / 8; y < map_height * 7 / 8; y++)
	{
		map_matrix[map_width / 2][y] = 1;
	}
	
	int x_start, y_start, x_finish, y_finish;

	//assign random values
	switch (rand() % 8)
	{
	case 0: x_start = 0; y_start = 0; x_finish = map_width - 1; y_finish = map_height - 1; break;
	case 1: x_start = 0; y_start = map_height - 1; x_finish = map_width - 1; y_finish = 0; break;
	case 2: x_start = map_width / 2 - 1; y_start = map_height / 2 - 1; x_finish = map_width / 2 + 1; y_finish = map_height / 2 + 1; break;
	case 3: x_start = map_width / 2 - 1; y_start = map_height / 2 + 1; x_finish = map_width / 2 + 1; y_finish = map_height / 2 - 1; break;
	case 4: x_start = map_width / 2 - 1; y_start = 0; x_finish = map_width / 2 + 1; y_finish = map_height - 1; break;
	case 5: x_start = map_width / 2 + 1; y_start = map_height - 1; x_finish = map_width / 2 - 1; y_finish = 0; break;
	case 6: x_start = 0; y_start = map_height / 2 - 1; x_finish = map_width - 1; y_finish = map_height / 2 + 1; break;
	case 7: x_start = map_width - 1; y_start = map_height / 2 + 1; x_finish = 0; y_finish = map_height / 2 - 1; break;
	}

	cout << "Start: " << x_start << "," << y_start << endl;
	cout << "Finish: " << x_finish << "," << y_finish << endl;
	clock_t start = clock();
	string route = findPath(x_start, y_start, x_finish, y_finish);
	if (route == "") cout << "There is no way from the start to the finish field" << endl;
	clock_t end = clock();
	double time_elapsed = double(end - start);
	cout << "Path calculated in (ms): " << time_elapsed << endl;
	cout << "Route:" << endl;
	cout << route << endl << endl;

	if (route.length() > 0)
	{
		int neighbors_index; 
		char path_char;
		int x = x_start;
		int y = y_start;

		map_matrix[x][y] = 2;
		for (int i = 0; i < route.length(); i++)
		{
			path_char = route.at(i);
			//convert string to integer
			neighbors_index = atoi(&path_char);
			x = x + horizontal_neighbors[neighbors_index];
			y = y + vertical_neighbors[neighbors_index];
			map_matrix[x][y] = 3;
		}
		map_matrix[x][y] = 4;

		// display the map with the path
		for (int y = 0; y < map_height; y++)
		{
			for (int x = 0; x < map_width; x++)
				if (map_matrix[x][y] == 0)
					cout << ".";
				else if (map_matrix[x][y] == 1)
					cout << "O"; //obstacle
				else if (map_matrix[x][y] == 2)
					cout << "S"; //start
				else if (map_matrix[x][y] == 3)
					cout << "R"; //route
				else if (map_matrix[x][y] == 4)
					cout << "F"; //finish
			cout << endl;
		}
	}
	getchar(); 
	return(0);
}
