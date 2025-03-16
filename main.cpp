#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <cmath>
#include <set>
#include <queue>
#include <algorithm>
#include <fstream>
#include <limits>

// список рёбер
std::vector<std::tuple<std::string, std::string, int>> edges;

// матрица смежности
int **matrix;

// количество городов
const int numTowns = 10;

// сопоставление городов с индексами
std::map<std::string, int> townToIndex;

// отображение индексов в названия городов
std::map<int, std::string> indexToTown;

// позиции городов на экране
std::vector<sf::Vector2f> positions;

// хранения путей и вершин
std::set<std::pair<int, int>> pathEdges;
std::set<int> pathVertices;
std::vector<int> startVertices;
int endVertex = -1;
int passThroughVertex = -1;

// хранение полного пути для вывода в файл
std::vector<int> fullPath;

// глобальная переменная для общего километража
int totalDistance = 0;

// цвета
const sf::Color EdgeColor(3, 252, 169);
const sf::Color StartVertexColor(34, 139, 34);
const sf::Color EndVertexColor(sf::Color::Black);
const sf::Color IntermediateVertexColor(3, 165, 252);
const sf::Color PassThroughVertexColor(240, 252, 3);

void addEdge(std::string from, std::string to, int data) {
    edges.push_back({from, to, data});
}

// принтим рёбра
void printEdge() {
    std::cout << "Список рёбер:" << std::endl;
    for (const auto& edge : edges) {
        std::cout << " {" << std::get<0>(edge) << ", " << std::get<1>(edge)
                  << ", " << std::get<2>(edge) << "}";
    }
    std::cout << std::endl;
}

// строим матрицу
void buildMatrix() {
    // инициализация матрицы нулями
    for (int i = 0; i < numTowns; ++i) {
        for (int j = 0; j < numTowns; ++j) {
            matrix[i][j] = 0;
        }
    }

    // заполнение матрицы
    for (const auto& edge : edges) {
        std::string from = std::get<0>(edge);
        std::string to = std::get<1>(edge);
        /*int data = std::get<2>(edge);*/

        int fromIndex = townToIndex[from];
        int toIndex = townToIndex[to];

        // 1 для пересечений
        matrix[fromIndex][toIndex] = 1;
        matrix[toIndex][fromIndex] = 1;
    }
}

// вывод городов (чисто для таблицы)
void printTowns(const std::vector<std::string>& towns) {
    std::cout << "Города и их индексы:" << std::endl;
    for (int i = 0; i < (int)towns.size(); ++i) {
        std::cout << i + 1 << ": " << towns[i] << std::endl;
    }
    std::cout << std::endl;
}

// вывод матрицы
void printMatrix() {
    std::cout << "Матрица смежности:" << std::endl;

    std::cout << "   ";
    for (int i = 0; i < numTowns; ++i) {
        std::cout << i + 1 << " ";
    }
    std::cout << std::endl;

    std::cout << "   ";
    for (int i = 0; i < numTowns; ++i) {
        std::cout << "_ ";
    }
    std::cout << std::endl;

    for (int i = 0; i < numTowns; ++i) {
        std::cout << i + 1 << "| ";
        for (int j = 0; j < numTowns; ++j) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// рисуем чб граф
int drawGraph(sf::RenderWindow& window, const std::vector<std::string>& towns) {
    float radius = 15.f; // радиус круга (вершины)

    // отрисовка рёбер
    for (const auto& edge : edges) {
        std::string from = std::get<0>(edge);
        std::string to = std::get<1>(edge);

        int fromIndex = townToIndex[from];
        int toIndex = townToIndex[to];

        sf::Vector2f start = positions[fromIndex];
        sf::Vector2f end = positions[toIndex];

        sf::RectangleShape line;

        // длина и угол линии
        float length = std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
        line.setSize(sf::Vector2f(length, 4.f));

        // всегда устанавливаем серый цвет для рёбер
        line.setFillColor(sf::Color(100, 100, 100));

        line.setPosition(start);
        float angle = std::atan2(end.y - start.y, end.x - start.x) * 180 / 3.14159265;
        line.setRotation(angle);

        window.draw(line);
    }

    // отрисовка вершин, смещения для текста относительно вершины
    std::vector<sf::Vector2f> labelOffsets = {
            {-55.f, -15.f}, // Amber
            {-55.f, -30.f}, // Svetlogorsk
            {-65.f, 20.f},  // Primorsk
            {-50.f, 15.f},  // Baltiysk
            {-13.f, 30.f},  // Kaliningrad
            {-85.f, -20.f}, // Kolosovka
            {5.f, -30.f},   // Pionersky
            {10.f, -30.f},  // Zelenogradsk
            {15.f, 10.f},   // Guryevsk
            {15.f, 20.f}    // Gvardeysk
    };

    sf::Font font;
    if (!font.loadFromFile("/System/Library/Fonts/Supplemental/Arial.ttf")) {
        std::cerr << "Ошибка загрузки шрифта!" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < towns.size(); ++i) {
        sf::CircleShape circle(radius);
        circle.setOrigin(radius, radius);
        circle.setPosition(positions[i]);

        // определяем цвет вершины
        circle.setFillColor(sf::Color::Red);

        // рисуем вершину
        window.draw(circle);

        // рисуем подписи вершин
        sf::Text text;
        text.setFont(font);
        text.setString(towns[i]);
        text.setCharacterSize(14); // размер текста
        text.setFillColor(sf::Color::Black);

        text.setPosition(
                positions[i].x + labelOffsets[i].x,
                positions[i].y + labelOffsets[i].y
        );

        // рисуем текст
        window.draw(text);
    }
    return 0;
}

int drawPrintedGraph(sf::RenderWindow& window, const std::vector<std::string>& towns) {
    float radius = 15.f; // радиус вершины

    // отрисовка рёбер
    for (const auto& edge : edges) {
        std::string from = std::get<0>(edge);
        std::string to = std::get<1>(edge);

        int fromIndex = townToIndex[from];
        int toIndex = townToIndex[to];

        sf::Vector2f start = positions[fromIndex];
        sf::Vector2f end = positions[toIndex];

        sf::RectangleShape line;

        // длина и угол линии
        float length = std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
        line.setSize(sf::Vector2f(length, 4.f));

        int u = fromIndex;
        int v = toIndex;
        if (u > v) std::swap(u, v);
        if (pathEdges.find({u, v}) != pathEdges.end()) {
            line.setFillColor(EdgeColor);
        } else {
            line.setFillColor(sf::Color(100, 100, 100));
        }

        line.setPosition(start);
        float angle = std::atan2(end.y - start.y, end.x - start.x) * 180 / 3.14159265;
        line.setRotation(angle);

        window.draw(line);
    }

    // отрисовка вершин
    std::vector<sf::Vector2f> labelOffsets = {
            {-55.f, -15.f}, // Amber
            {-55.f, -30.f}, // Svetlogorsk
            {-65.f, 20.f},  // Primorsk
            {-50.f, 15.f},  // Baltiysk
            {-13.f, 30.f},  // Kaliningrad
            {-85.f, -20.f}, // Kolosovka
            {5.f, -30.f},   // Pionersky
            {10.f, -30.f},  // Zelenogradsk
            {15.f, 10.f},   // Guryevsk
            {15.f, 20.f}    // Gvardeysk
    };

    sf::Font font;
    if (!font.loadFromFile("/System/Library/Fonts/Supplemental/Arial.ttf")) {
        std::cerr << "Ошибка загрузки шрифта!" << std::endl;
        return -1;
    }
    // рисуем вершины
    for (size_t i = 0; i < towns.size(); ++i) {
        sf::CircleShape circle(radius);
        circle.setOrigin(radius, radius);
        circle.setPosition(positions[i]);

        if (std::find(startVertices.begin(), startVertices.end(), (int)i) != startVertices.end()) {
            circle.setFillColor(StartVertexColor);
        } else if ((int)i == endVertex) {
            circle.setFillColor(EndVertexColor);
        } else if ((int)i == passThroughVertex) {
            circle.setFillColor(PassThroughVertexColor);
        } else if (pathVertices.find((int)i) != pathVertices.end()) {
            circle.setFillColor(IntermediateVertexColor);
        } else {
            circle.setFillColor(sf::Color::Red); // обычная вершина
        }

        window.draw(circle);

        sf::Text text;
        text.setFont(font);
        text.setString(towns[i]);
        text.setCharacterSize(14);
        text.setFillColor(sf::Color::Black);

        text.setPosition(
                positions[i].x + labelOffsets[i].x,
                positions[i].y + labelOffsets[i].y
        );

        window.draw(text);
    }

    // общий километраж
    sf::Text distanceText;
    distanceText.setFont(font);
    distanceText.setString(std::to_string(totalDistance) + " KM");
    distanceText.setCharacterSize(18);
    distanceText.setFillColor(sf::Color::Black);

    sf::FloatRect textRect = distanceText.getLocalBounds();
    distanceText.setOrigin(textRect.width, 0);
    distanceText.setPosition(window.getSize().x - 10.f, 10.f);

    window.draw(distanceText);

    return 0;
}

const int INF = std::numeric_limits<int>::max();


// вектор бул в ключ
std::string visitedEdgesToString(const std::vector<bool>& vec) {
    std::string res;
    res.reserve(vec.size());
    for (bool b : vec) {
        res.push_back(b ? '1' : '0');
    }
    return res;
}

// Дейкстра
std::pair<int, std::vector<int>> Dijkstra(
        const std::vector<std::tuple<std::string, std::string, int>>& edges,
        const std::map<std::string, int>& townToIndex,
        int numTowns,
        int start,
        int end,
        int passThroughVertex,
        const std::map<std::pair<int, int>, int>& edgeToID) {

    // создание списка смежности
    std::vector<std::vector<std::pair<int, int>>> adjList(numTowns);
    for (const auto& edge : edges) {
        int from = townToIndex.at(std::get<0>(edge));
        int to = townToIndex.at(std::get<1>(edge));
        int weight = std::get<2>(edge);
        adjList[from].push_back({to, weight});
        adjList[to].push_back({from, weight});
    }

    // состояние
    struct State {
        int dist;
        int node;
        bool passedThrough;
        std::vector<bool> visitedEdges;

        bool operator>(const State& other) const {
            return dist > other.dist;
        }
    };

    std::vector<bool> initialVisitedEdges(edges.size(), false);
    bool initialPassed = (start == passThroughVertex);

    State initialState = {0, start, initialPassed, initialVisitedEdges};

    // очередь с приоритетом
    std::priority_queue<State, std::vector<State>, std::greater<>> pq;
    pq.push(initialState);

    // восстановка пути

    using Key = std::tuple<int, bool, std::string>; // ключ: node, passedThrough, visitedEdgesString
    std::map<Key, int> distances;
    std::map<Key, Key> previous;

    Key startKey = {start, initialPassed, visitedEdgesToString(initialVisitedEdges)};
    distances[startKey] = 0;

    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();
        std::string curVisitedStr = visitedEdgesToString(current.visitedEdges);
        Key curKey = {current.node, current.passedThrough, curVisitedStr};
        if (distances[curKey] < current.dist) {
            continue;
        }

        // проверка условия достижения
        if (current.node == end && current.passedThrough) {
            // восстанавливаем путь
            std::vector<int> path;
            Key k = curKey;
            while (previous.find(k) != previous.end()) {
                path.push_back(std::get<0>(k));
                k = previous[k];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return {current.dist, path};
        }

        // перебор соседей
        for (auto& [neighbor, weight] : adjList[current.node]) {
            int from = std::min(current.node, neighbor);
            int to = std::max(current.node, neighbor);
            int edgeID = edgeToID.at({from, to});

            // проверка ребра
            if (current.visitedEdges[edgeID]) continue;

            State newState = current;
            newState.dist = current.dist + weight;
            newState.node = neighbor;
            newState.passedThrough = current.passedThrough || (neighbor == passThroughVertex);
            newState.visitedEdges[edgeID] = true;

            std::string newVisitedStr = visitedEdgesToString(newState.visitedEdges);
            Key newKey = {newState.node, newState.passedThrough, newVisitedStr};

            if (distances.find(newKey) == distances.end() || distances[newKey] > newState.dist) {
                distances[newKey] = newState.dist;
                previous[newKey] = curKey;
                pq.push(newState);
            }
        }
    }

    return {INF, {}};
}

// функция поиска пути через обязательную вершину
void findPathThroughVertex(std::vector<std::tuple<std::string, std::string, int>>& edges, const std::map<std::string, int>& townToIndex,
                           int numTowns,
                           const std::map<std::pair<int, int>, int>& edgeToID) {
    int startVertexLocal = townToIndex.at("Amber");
    int endVertexLocal = townToIndex.at("Kaliningrad");
    passThroughVertex = townToIndex.at("Kolosovka");
    /* "Amber", "Svetlogorsk", "Primorsk", "Baltiysk", "Kaliningrad",
            "Kolosovka", "Pionersky", "Zelenogradsk", "Guryevsk", "Gvardeysk"*/

    auto [distance, path] = Dijkstra(edges, townToIndex, numTowns, startVertexLocal, endVertexLocal, passThroughVertex, edgeToID);

    if (path.empty() || path.back() != endVertexLocal) {
        std::cout << "Путь от стартовой до конечной вершины, проходящий через обязательную вершину, не найден." << std::endl;
        return;
    }

    fullPath = path;

    // Запоминаем рёбра пути
    for (size_t i = 1; i < fullPath.size(); ++i) {
        int from = fullPath[i - 1];
        int to = fullPath[i];
        if (from > to) std::swap(from, to);
        pathEdges.insert({from, to});
    }

    // запоминаем вершины пути
    for (int vertex : fullPath) {
        pathVertices.insert(vertex);
    }

    // общий километраж
    totalDistance = 0;
    for (size_t i = 1; i < fullPath.size(); ++i) {
        int from = fullPath[i - 1];
        int to = fullPath[i];
        int weight = 0;

        std::string fromTown = indexToTown[from];
        std::string toTown = indexToTown[to];

        for (const auto& edge : edges) {
            if ((std::get<0>(edge) == fromTown && std::get<1>(edge) == toTown) ||
                (std::get<0>(edge) == toTown && std::get<1>(edge) == fromTown)) {
                weight = std::get<2>(edge);
                break;
            }
        }

        totalDistance += weight;
    }

    // стартовая и конечная вершина для окраски
    startVertices.push_back(startVertexLocal);
    endVertex = endVertexLocal;
}

int main() {
    // список городов
    std::vector<std::string> towns = {
            "Amber", "Svetlogorsk", "Primorsk", "Baltiysk", "Kaliningrad",
            "Kolosovka", "Pionersky", "Zelenogradsk", "Guryevsk", "Gvardeysk"
    };

    positions = {
            {150, 500},  // Amber
            {250, 300},  // Svetlogorsk
            {200, 600},  // Primorsk
            {200, 700},  // Baltiysk
            {350, 500},  // Kaliningrad
            {300, 400},  // Kolosovka
            {400, 350},  // Pionersky
            {550, 350},  // Zelenogradsk
            {500, 500},  // Guryevsk
            {550, 600}   // Gvardeysk
    };

    for (int i = 0; i < (int)towns.size(); ++i) {
        townToIndex[towns[i]] = i;
        indexToTown[i] = towns[i];
    }

    matrix = new int*[numTowns];
    for (int i = 0; i < numTowns; ++i) {
        matrix[i] = new int[numTowns];
    }

    // добавляем рёбра
    addEdge("Amber", "Svetlogorsk", 2);
    addEdge("Amber", "Primorsk", 2);

    addEdge("Primorsk", "Kolosovka", 1);
    addEdge("Primorsk", "Baltiysk", 14);
    addEdge("Primorsk", "Kaliningrad", 1);

    addEdge("Baltiysk", "Kaliningrad", 45);

    addEdge("Kaliningrad", "Kolosovka", 5);
    addEdge("Kaliningrad", "Pionersky", 44);
    addEdge("Kaliningrad", "Zelenogradsk", 33);
    addEdge("Kaliningrad", "Guryevsk", 13);
    addEdge("Kaliningrad", "Gvardeysk", 53);

    addEdge("Kolosovka", "Svetlogorsk", 1);

    addEdge("Svetlogorsk", "Pionersky", 5);
    addEdge("Svetlogorsk", "Kaliningrad", 1);

    addEdge("Pionersky", "Zelenogradsk", 24);

    addEdge("Zelenogradsk", "Guryevsk", 25);

    addEdge("Guryevsk", "Gvardeysk", 37);

    // построение матрицы
    buildMatrix();

    // печать списка рёбер
    printEdge();

    // печать матрицы
    printMatrix();

    // печать индексов городов
    printTowns(towns);

    // ID рёбрам
    std::map<std::pair<int, int>, int> edgeToID;
    int edgeID = 0;
    for (const auto& edge : edges) {
        int from = townToIndex[std::get<0>(edge)];
        int to = townToIndex[std::get<1>(edge)];
        if (from > to) std::swap(from, to);
        edgeToID[{from, to}] = edgeID++;
    }

    findPathThroughVertex(edges, townToIndex, numTowns, edgeToID);

    // окно
    sf::RenderWindow window(sf::VideoMode(800, 800), "Graph");

    bool isColored = false;
    bool isPush = false;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space && !isPush) {
                    isPush = true;
                    isColored = !isColored;

                    std::ofstream vertexFile("/Users/majestic/Desktop/lab6SFML/outVertex.txt");
                    std::ofstream edgeFile("/Users/majestic/Desktop/lab6SFML/outEdge.txt");

                    if (vertexFile.is_open() && edgeFile.is_open()) {
                        // вывод списка вершин
                        for (int node : fullPath) {
                            vertexFile << indexToTown[node] << std::endl;
                        }

                        // вывод списка рёбер
                        for (size_t i = 1; i < fullPath.size(); ++i) {
                            int from = fullPath[i - 1];
                            int to = fullPath[i];
                            int weight = 0;
                            std::string fromTown = indexToTown[from];
                            std::string toTown = indexToTown[to];

                            for (const auto& edge : edges) {
                                if ((std::get<0>(edge) == fromTown && std::get<1>(edge) == toTown) ||
                                    (std::get<0>(edge) == toTown && std::get<1>(edge) == fromTown)) {
                                    weight = std::get<2>(edge);
                                    break;
                                }
                            }

                            edgeFile << fromTown << " -> " << toTown << " : " << weight << std::endl;
                        }

                        vertexFile.close();
                        edgeFile.close();
                        std::cout << "Кратчайший маршрут записан в файлы outVertex.txt и outEdge.txt" << std::endl;
                    } else {
                        std::cerr << "Не удалось открыть файлы для записи пути." << std::endl;
                    }
                }
            }
            else if (event.type == sf::Event::KeyReleased) {
                isPush = false;
            }

            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color::White);
        if (isColored) {
            drawPrintedGraph(window, towns);
        } else {
            drawGraph(window, towns);
        }
        window.display();
    }

    for (int i = 0; i < numTowns; ++i) {
        delete[] matrix[i];
    }
    delete[] matrix;

    return 0;
}
