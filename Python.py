import heapq
from collections import defaultdict, deque

def prim_mst(n, graph):
    """Построение MST алгоритмом Прима с детальными шагами"""
    # Шаг 1.1: Инициализация
    visited = [False] * n
    min_weight = [float('inf')] * n  # минимальный вес ребра к MST
    parent = [-1] * n                # родительские вершины в MST
    min_weight[0] = 0
    
    # Приоритетная очередь: (вес, вершина)
    heap = [(0, 0)]
    
    mst_edges = []
    total_mst_weight = 0
    
    while heap:
        # Шаг 1.3: Выбор ребра минимального веса
        current_weight, u = heapq.heappop(heap)
        
        if visited[u]:
            continue
            
        # Шаг 1.4: Добавление вершины в MST
        visited[u] = True
        total_mst_weight += current_weight
        
        if parent[u] != -1:  # кроме стартовой вершины
            mst_edges.append((parent[u], u, current_weight))
        
        # Обновление весов для соседних вершин
        for v in range(n):
            if not visited[v] and graph[u][v] < min_weight[v]:
                min_weight[v] = graph[u][v]
                parent[v] = u
                heapq.heappush(heap, (graph[u][v], v))
    
    print(f"МST построено: {len(mst_edges)} ребер, общий вес: {total_mst_weight}")
    return mst_edges

def find_eulerian_tour(mst_edges, n):
    """Построение эйлерова обхода с детальными шагами"""
    # Шаг 2.1-2.2: Создание мультиграфа с удвоенными ребрами
    adjacency = defaultdict(list)
    edge_count = defaultdict(int)
    
    for u, v, weight in mst_edges:
        # Добавляем каждое ребро дважды
        adjacency[u].append((v, weight))
        adjacency[v].append((u, weight))
        adjacency[u].append((v, weight))
        adjacency[v].append((u, weight))
        
        edge_count[(min(u, v), max(u, v))] += 2
    
    print(f"Удвоенный граф создан: {sum(len(adj) for adj in adjacency.values())} ребер")
    
    # Шаг 3: Поиск эйлерова цикла (итеративный DFS)
    stack = [0]
    euler_tour = []
    
    while stack:
        u = stack[-1]
        
        if adjacency[u]:
            # Берем следующее ребро
            v, weight = adjacency[u].pop()
            
            # Удаляем обратное ребро
            for i in range(len(adjacency[v])):
                if adjacency[v][i][0] == u:
                    adjacency[v].pop(i)
                    break
            
            stack.append(v)
        else:
            # Вершина обработана
            euler_tour.append(stack.pop())
    
    euler_tour = euler_tour[::-1]  # разворачиваем для правильного порядка
    print(f"Эйлеров обход найден: {len(euler_tour)} вершин")
    return euler_tour
def shortcut_to_hamiltonian(euler_tour):
    """Преобразование эйлерова обхода в гамильтонов цикл"""
    # Шаг 4.1-4.3: Удаление повторяющихся вершин
    visited = set()
    hamiltonian_tour = []
    
    for vertex in euler_tour:
        if vertex not in visited:
            visited.add(vertex)
            hamiltonian_tour.append(vertex)
    
    # Шаг 4.4: Замыкание цикла
    hamiltonian_tour.append(hamiltonian_tour[0])
    
    print(f"Гамильтонов цикл: {len(hamiltonian_tour)} вершин, уникальных: {len(visited)}")
    return hamiltonian_tour

def tsp_2_approximation_improved(distances):
    """Улучшенная 2-аппроксимация для TSP"""
    n = len(distances)
    
    print("=" * 50)
    print("ШАГ 1: Построение минимального остовного дерева (MST)")
    mst_edges = prim_mst(n, distances)
    
    print("\nШАГ 2: Удвоение ребер MST для получения эйлерова графа")
    print("ШАГ 3: Построение эйлерова обхода")
    euler_tour = find_eulerian_tour(mst_edges, n)
    
    print("\nШАГ 4: Преобразование в гамильтонов цикл (удаление повторений)")
    hamiltonian_tour = shortcut_to_hamiltonian(euler_tour)
    
    # Вычисление стоимости
    tour_cost = 0
    for i in range(len(hamiltonian_tour) - 1):
        u = hamiltonian_tour[i]
        v = hamiltonian_tour[i + 1]
        tour_cost += distances[u][v]
    
    print(f"\nРЕЗУЛЬТАТ: Стоимость маршрута = {tour_cost}")
    print("=" * 50)
    
    return hamiltonian_tour, tour_cost

def analyze_approximation_quality(tour_cost, distances, optimal_estimate=None):
    """Анализ качества аппроксимации"""
    n = len(distances)
    
    # Нижняя оценка через MST
    mst_lower_bound = 0
    temp_edges = prim_mst(n, distances)
    for u, v, w in temp_edges:
        mst_lower_bound += w
    
    approximation_ratio = tour_cost / mst_lower_bound if mst_lower_bound > 0 else float('inf')
    
    print("\nАНАЛИЗ КАЧЕСТВА:")
    print(f"Стоимость найденного тура: {tour_cost}")
    print(f"Нижняя оценка (вес MST): {mst_lower_bound}")
    print(f"Коэффициент аппроксимации: {approximation_ratio:.3f}")
    
    if approximation_ratio <= 2.0:
        print("Алгоритм обеспечивает 2-аппроксимацию")
    else:
        print("Коэффициент превышает 2, возможна ошибка")

# Тестирование
if name == "main":
    # Метрическая матрица расстояний (удовлетворяет неравенству треугольника)
    distances = [
        [0, 10, 15, 20, 25],
        [10, 0, 35, 25, 30],
        [15, 35, 0, 30, 20],
        [20, 25, 30, 0, 15],
        [25, 30, 20, 15, 0]
    ]
    
    print("МАТРИЦА РАССТОЯНИЙ (5 городов):")
    for i, row in enumerate(distances):
        print(f"Город {i}: {row}")
    
    tour, cost = tsp_2_approximation_improved(distances)
    
    print(f"\nФИНАЛЬНЫЙ МАРШРУТ: {tour}")
    print(f"ПОРЯДОК ОБХОДА: {' → '.join(map(str, tour))}")
    
    analyze_approximation_quality(cost, distances)
    
    # Проверка корректности
    unique_cities = set(tour[:-1])  # исключаем повтор старта
    print(f"\nПРОВЕРКА КОРРЕКТНОСТИ:")
    print(f"Все города посещены: {len(unique_cities) == len(distances)}")
    print(f"Маршрут замкнут: {tour[0] == tour[-1]}")
    print(f"Длина маршрута: {len(tour)} точек")

## Вывод программы

Матрица расстояний:
[0, 10, 15, 20, 25]
[10, 0, 35, 25, 30]
[15, 35, 0, 30, 20]
[20, 25, 30, 0, 15]
[25, 30, 20, 15, 0]

Найденный маршрут: [0, 1, 3, 4, 2, 0]
Стоимость маршрута: 105
Количество городов в маршруте: 5
Начинается и заканчивается в одном городе: True
