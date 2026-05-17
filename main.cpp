#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <unordered_map>

const int WIDTH = 800;
const int HEIGHT = 800;
const int ROWS = 40;
const int COLS = 40;
const int CELL_SIZE = WIDTH / COLS;

enum State { EMPTY, WALL, START, END, VISITED_F, VISITED_B, PATH };
enum AlgoType { NONE, BFS_ALGO, DFS_ALGO, DIJKSTRA_ALGO, BI_ASTAR_ALGO };

struct Node {
    int id, row, col;
    float costF, costB, priority;
    int parentF, parentB;
    State state;

    Node(int r = 0, int c = 0) : id(r * COLS + c), row(r), col(c), 
        costF(std::numeric_limits<float>::infinity()), costB(std::numeric_limits<float>::infinity()), 
        priority(0), parentF(-1), parentB(-1), state(EMPTY) {}

    sf::RectangleShape getShape() const {
        sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1.0f, CELL_SIZE - 1.0f));
        cell.setPosition(col * CELL_SIZE, row * CELL_SIZE);
        switch (state) {
            case WALL: cell.setFillColor(sf::Color::Black); break;
            case START: cell.setFillColor(sf::Color::Green); break;
            case END: cell.setFillColor(sf::Color::Red); break;
            case VISITED_F: cell.setFillColor(sf::Color::Cyan); break;
            case VISITED_B: cell.setFillColor(sf::Color::Magenta); break;
            case PATH: cell.setFillColor(sf::Color::Yellow); break;
            default: cell.setFillColor(sf::Color::White);
        }
        return cell;
    }
};

struct CompareNode {
    bool operator()(const std::pair<float, int>& a, const std::pair<float, int>& b) {
        return a.first > b.first;
    }
};

std::vector<std::pair<int, int>> directions = {{-1,0}, {1,0}, {0,-1}, {0,1}};

bool isValid(int r, int c) { return r >= 0 && r < ROWS && c >= 0 && c < COLS; }
float heuristic(int r1, int c1, int r2, int c2) { return std::abs(r1 - r2) + std::abs(c1 - c2); }

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "High-Performance Pathfinding");
    window.setFramerateLimit(60);

    std::vector<Node> grid(ROWS * COLS);
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            grid[i * COLS + j] = Node(i, j);

    int startId = -1, endId = -1;
    AlgoType currentAlgo = NONE;
    bool isRunning = false;
    bool pathFound = false;
    int meetingNode = -1;

    std::queue<int> q;
    std::vector<int> stack;
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, CompareNode> pq;
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, CompareNode> pqF, pqB;

    auto resetSearchState = [&]() {
        isRunning = false; pathFound = false; meetingNode = -1;
        q = std::queue<int>(); stack.clear();
        pq = decltype(pq)(); pqF = decltype(pqF)(); pqB = decltype(pqB)();
        for (auto& n : grid) {
            if (n.state != WALL && n.state != START && n.state != END) n.state = EMPTY;
            n.costF = std::numeric_limits<float>::infinity();
            n.costB = std::numeric_limits<float>::infinity();
            n.parentF = -1; n.parentB = -1;
        }
    };

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed && startId != -1 && endId != -1 && !isRunning) {
                resetSearchState();
                if (event.key.code == sf::Keyboard::B) {
                    currentAlgo = BFS_ALGO; isRunning = true;
                    q.push(startId); grid[startId].costF = 0;
                } else if (event.key.code == sf::Keyboard::D) {
                    currentAlgo = DFS_ALGO; isRunning = true;
                    stack.push_back(startId);
                } else if (event.key.code == sf::Keyboard::K) {
                    currentAlgo = DIJKSTRA_ALGO; isRunning = true;
                    pq.push({0, startId}); grid[startId].costF = 0;
                } else if (event.key.code == sf::Keyboard::A) {
                    currentAlgo = BI_ASTAR_ALGO; isRunning = true;
                    pqF.push({heuristic(grid[startId].row, grid[startId].col, grid[endId].row, grid[endId].col), startId});
                    pqB.push({heuristic(grid[endId].row, grid[endId].col, grid[startId].row, grid[startId].col), endId});
                    grid[startId].costF = 0; grid[endId].costB = 0;
                } else if (event.key.code == sf::Keyboard::R) {
                    startId = -1; endId = -1; currentAlgo = NONE;
                    for (int i = 0; i < ROWS * COLS; ++i) grid[i] = Node(i / COLS, i % COLS);
                }
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && !isRunning) {
            int x = sf::Mouse::getPosition(window).x / CELL_SIZE;
            int y = sf::Mouse::getPosition(window).y / CELL_SIZE;
            if (isValid(y, x)) {
                int id = y * COLS + x;
                if (startId == -1) { startId = id; grid[id].state = START; }
                else if (endId == -1 && id != startId) { endId = id; grid[id].state = END; }
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Right) && !isRunning) {
            int x = sf::Mouse::getPosition(window).x / CELL_SIZE;
            int y = sf::Mouse::getPosition(window).y / CELL_SIZE;
            if (isValid(y, x)) {
                int id = y * COLS + x;
                if (id != startId && id != endId) grid[id].state = WALL;
            }
        }

        if (isRunning && !pathFound) {
            for(int step = 0; step < 5 && isRunning && !pathFound; ++step) {
                if (currentAlgo == BFS_ALGO) {
                    if (q.empty()) { isRunning = false; break; }
                    int curr = q.front(); q.pop();
                    if (curr == endId) { pathFound = true; meetingNode = curr; break; }
                    for (auto [dr, dc] : directions) {
                        int nr = grid[curr].row + dr, nc = grid[curr].col + dc;
                        if (isValid(nr, nc)) {
                            int nid = nr * COLS + nc;
                            if (grid[nid].state != WALL && grid[nid].costF == std::numeric_limits<float>::infinity()) {
                                grid[nid].costF = grid[curr].costF + 1;
                                grid[nid].parentF = curr;
                                if (nid != endId) grid[nid].state = VISITED_F;
                                q.push(nid);
                            }
                        }
                    }
                } else if (currentAlgo == DFS_ALGO) {
                    if (stack.empty()) { isRunning = false; break; }
                    int curr = stack.back(); stack.pop_back();
                    if (curr == endId) { pathFound = true; meetingNode = curr; break; }
                    if (curr != startId) grid[curr].state = VISITED_F;
                    for (auto [dr, dc] : directions) {
                        int nr = grid[curr].row + dr, nc = grid[curr].col + dc;
                        if (isValid(nr, nc)) {
                            int nid = nr * COLS + nc;
                            if ((grid[nid].state == EMPTY || grid[nid].state == END) && grid[nid].parentF == -1) {
                                grid[nid].parentF = curr;
                                stack.push_back(nid);
                            }
                        }
                    }
                } else if (currentAlgo == DIJKSTRA_ALGO) {
                    if (pq.empty()) { isRunning = false; break; }
                    int curr = pq.top().second; pq.pop();
                    if (curr == endId) { pathFound = true; meetingNode = curr; break; }
                    for (auto [dr, dc] : directions) {
                        int nr = grid[curr].row + dr, nc = grid[curr].col + dc;
                        if (isValid(nr, nc)) {
                            int nid = nr * COLS + nc;
                            if (grid[nid].state != WALL && grid[curr].costF + 1 < grid[nid].costF) {
                                grid[nid].costF = grid[curr].costF + 1;
                                grid[nid].parentF = curr;
                                if (nid != endId) grid[nid].state = VISITED_F;
                                pq.push({grid[nid].costF, nid});
                            }
                        }
                    }
                } else if (currentAlgo == BI_ASTAR_ALGO) {
                    if (pqF.empty() || pqB.empty()) { isRunning = false; break; }
                    
                    int currF = pqF.top().second; pqF.pop();
                    if (grid[currF].parentB != -1 || currF == endId) { pathFound = true; meetingNode = currF; break; }
                    for (auto [dr, dc] : directions) {
                        int nr = grid[currF].row + dr, nc = grid[currF].col + dc;
                        if (isValid(nr, nc)) {
                            int nid = nr * COLS + nc;
                            if (grid[nid].state != WALL && grid[currF].costF + 1 < grid[nid].costF) {
                                grid[nid].costF = grid[currF].costF + 1;
                                grid[nid].parentF = currF;
                                if (nid != startId && nid != endId) grid[nid].state = VISITED_F;
                                pqF.push({grid[nid].costF + heuristic(nr, nc, grid[endId].row, grid[endId].col), nid});
                            }
                        }
                    }

                    if(pathFound) break;

                    int currB = pqB.top().second; pqB.pop();
                    if (grid[currB].parentF != -1 || currB == startId) { pathFound = true; meetingNode = currB; break; }
                    for (auto [dr, dc] : directions) {
                        int nr = grid[currB].row + dr, nc = grid[currB].col + dc;
                        if (isValid(nr, nc)) {
                            int nid = nr * COLS + nc;
                            if (grid[nid].state != WALL && grid[currB].costB + 1 < grid[nid].costB) {
                                grid[nid].costB = grid[currB].costB + 1;
                                grid[nid].parentB = currB;
                                if (nid != startId && nid != endId) grid[nid].state = VISITED_B;
                                pqB.push({grid[nid].costB + heuristic(nr, nc, grid[startId].row, grid[startId].col), nid});
                            }
                        }
                    }
                }
            }
        }

        if (pathFound && meetingNode != -1) {
            int curr = meetingNode;
            while (curr != -1 && curr != startId && curr != endId) {
                grid[curr].state = PATH;
                curr = grid[curr].parentF;
            }
            curr = meetingNode;
            while (curr != -1 && curr != startId && curr != endId) {
                grid[curr].state = PATH;
                curr = grid[curr].parentB;
            }
            isRunning = false; 
            meetingNode = -1;
        }

        window.clear();
        for (const auto& node : grid) window.draw(node.getShape());
        window.display();
    }
    return 0;
}
