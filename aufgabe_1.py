
import argparse
import yaml
import heapq


class AStarSearch:
    def __init__(self, problem, heuristic_info):
        self.graph = problem['graph']
        self.start = problem['start']
        self.end = problem['end']
        self.heuristic_info = heuristic_info

    def a_star(self, heuristic_mode='no'):
        heuristic = self._select_heuristic(heuristic_mode)
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        came_from = {}
        g_score = {node: float('inf') for node in self.graph}
        g_score[self.start] = 0

        expanded_nodes = 0
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == self.end:
                return self._reconstruct_solution(came_from, current, g_score[self.end], expanded_nodes, heuristic)
            expanded_nodes += 1
            for neighbor, cost in self.graph[current].items():
                tentative_g_score = g_score[current] + cost
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic[neighbor]
                    heapq.heappush(open_set, (f_score, neighbor))
        raise ValueError("No path found from start to goal.")


    def _select_heuristic(self, heuristic_mode):
        if heuristic_mode == 'no':
            return {node: 0.0 for node in self.graph}
        elif heuristic_mode == 'simple':
            return {node: self.heuristic_info[node]['line_of_sight_distance'] for node in self.graph}
        elif heuristic_mode == 'advanced':
            return {
                node: self.heuristic_info[node]['line_of_sight_distance'] +
                      0.5 * self.heuristic_info[node]['altitude_difference']
                for node in self.graph
            }
        else:
            raise ValueError("Invalid heuristic mode")

    def _reconstruct_solution(self, came_from, current, cost, expanded_nodes, heuristic):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return {
            'cost': cost,
            'path': path,
            'expanded_nodes': expanded_nodes,
            'heuristic': {node: heuristic[node] for node in self.graph}
        }


def load_problem(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    cities = data['problem']['cities']
    start = data['problem']['city_start']
    end = data['problem']['city_end']

    graph = {}
    for city in cities:
        connections = data['problem'][f'city_{city}']['connects_to']
        graph[city] = {k: float(v) for k, v in connections.items()}

    heuristic_info = {}
    for city in cities:
        heuristic_info[city] = {
            'line_of_sight_distance': data['additional_information'][f'city_{city}']['line_of_sight_distance'],
            'altitude_difference': data['additional_information'][f'city_{city}']['altitude_difference']
        }

    return {'graph': graph, 'start': start, 'end': end}, heuristic_info


def save_solution(file_name, solution):
    # Приведение всех чисел к float
    formatted_solution = {
        'solution': {
            'cost': float(solution['cost']),  # Приведение стоимости к float
            'expanded_nodes': int(solution['expanded_nodes']),  # Количество узлов — целое число
            'heuristic': {f"city_{key}": float(value) for key, value in solution['heuristic'].items()},
            'path': solution['path']  # Путь без изменений
        }
    }
    with open(file_name, 'w') as file:
        yaml.dump(formatted_solution, file)


def main():
    parser = argparse.ArgumentParser(description="Solve pathfinding problem with A*")
    parser.add_argument("input_file", help="Path to the YAML file describing the problem")
    args = parser.parse_args()

# Загрузка данных задачи и эвристической информации
    problem_data, heuristic_data = load_problem(args.input_file)
    solver = AStarSearch(problem_data, heuristic_data)

    # Конфигурация режимов эвристики и их соответствующих выходных файлов
    tasks = [
        {'output': 'aufgabe1-1.yaml', 'heuristic': 'no'},
        {'output': 'aufgabe1-2.yaml', 'heuristic': 'simple'},
        {'output': 'aufgabe1-3.yaml', 'heuristic': 'advanced'}
    ]

    # Выполнение поиска и сохранение результатов
    for task in tasks:
        solution = solver.a_star(heuristic_mode=task['heuristic'])
        save_solution(task['output'], solution)

if __name__ == "__main__":
    main()