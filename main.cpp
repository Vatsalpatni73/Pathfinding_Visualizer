// Pathfinding Visualizer using SFML and C++
// Tech Stack: C++, SFML, STL (vector, queue, priority_queue)

#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>

const int WIDTH = 800;
const int HEIGHT = 800;
const int ROWS = 40;
const int COLS = 40;
const int CELL_SIZE = WIDTH / COLS;

enum State { EMPTY, WALL, START, END, VISITED, PATH };

struct Node {
    int row, col;
    float cost;
    float priority;
    Node* parent;
    State state;

    Node(int r = 0, int c = 0) : row(r), col(c), cost(std::numeric_limits<float>::infinity()), priority(0), parent(nullptr), state(EMPTY) {}

    sf::RectangleShape getShape() const {
        sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
        cell.setPosition(col * CELL_SIZE, row * CELL_SIZE);
        switch (state) {
            case WALL: cell.setFillColor(sf::Color::Black); break;
            case START: cell.setFillColor(sf::Color::Green); break;
            case END: cell.setFillColor(sf::Color::Red); break;
            case VISITED: cell.setFillColor(sf::Color::Cyan); break;
            case PATH: cell.setFillColor(sf::Color::Yellow); break;
            default: cell.setFillColor(sf::Color::White);
        }
        return cell;
    }
};

std::vector<std::pair<int, int>> directions = {{-1,0},{1,0},{0,-1},{0,1}};

bool isValid(int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS;
}

void bfs(std::vector<std::vector<Node>>& grid, Node* start, Node* end) {
    std::queue<Node*> q;
    q.push(start);
    start->cost = 0;
    while (!q.empty()) {
        Node* current = q.front(); q.pop();
        if (current == end) break;
        for (auto [dr, dc] : directions) {
            int nr = current->row + dr;
            int nc = current->col + dc;
            if (isValid(nr, nc)) {
                Node& neighbor = grid[nr][nc];
                if (neighbor.state != WALL && neighbor.cost == std::numeric_limits<float>::infinity()) {
                    neighbor.cost = current->cost + 1;
                    neighbor.parent = current;
                    if (&neighbor != end) neighbor.state = VISITED;
                    q.push(&neighbor);
                }
            }
        }
    }
}

void dfs(Node* current, Node* end, std::vector<std::vector<Node>>& grid, bool& found) {
    if (!current || found || current == end) {
        found = current == end;
        return;
    }
    if (current->state != START) current->state = VISITED;
    for (auto [dr, dc] : directions) {
        int nr = current->row + dr;
        int nc = current->col + dc;
        if (isValid(nr, nc)) {
            Node& neighbor = grid[nr][nc];
            if ((neighbor.state == EMPTY || neighbor.state == END) && neighbor.parent == nullptr) {
                neighbor.parent = current;
                dfs(&neighbor, end, grid, found);
            }
        }
    }
}

void dijkstra(std::vector<std::vector<Node>>& grid, Node* start, Node* end) {
    auto cmp = [](Node* a, Node* b) { return a->priority > b->priority; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);
    start->cost = 0;
    pq.push(start);

    while (!pq.empty()) {
        Node* current = pq.top(); pq.pop();
        if (current == end) break;
        for (auto [dr, dc] : directions) {
            int nr = current->row + dr;
            int nc = current->col + dc;
            if (isValid(nr, nc)) {
                Node& neighbor = grid[nr][nc];
                float newCost = current->cost + 1;
                if (neighbor.state != WALL && newCost < neighbor.cost) {
                    neighbor.cost = newCost;
                    neighbor.priority = newCost;
                    neighbor.parent = current;
                    if (&neighbor != end) neighbor.state = VISITED;
                    pq.push(&neighbor);
                }
            }
        }
    }
}

void drawPath(Node* end) {
    Node* curr = end->parent;
    while (curr && curr->state != START) {
        curr->state = PATH;
        curr = curr->parent;
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Pathfinding Visualizer");
    std::vector<std::vector<Node>> grid(ROWS, std::vector<Node>(COLS));
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            grid[i][j] = Node(i, j);

    Node *start = nullptr, *end = nullptr;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::B && start && end) {
                    bfs(grid, start, end);
                    drawPath(end);
                } else if (event.key.code == sf::Keyboard::D && start && end) {
                    bool found = false;
                    dfs(start, end, grid, found);
                    if (found) drawPath(end);
                } else if (event.key.code == sf::Keyboard::K && start && end) {
                    dijkstra(grid, start, end);
                    drawPath(end);
                } else if (event.key.code == sf::Keyboard::R) {
                    start = nullptr;
                    end = nullptr;
                    for (auto& row : grid)
                        for (auto& node : row) {
                            node.state = EMPTY;
                            node.parent = nullptr;
                            node.cost = std::numeric_limits<float>::infinity();
                        }
                }
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            int x = sf::Mouse::getPosition(window).x / CELL_SIZE;
            int y = sf::Mouse::getPosition(window).y / CELL_SIZE;
            if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
                if (!start) {
                    start = &grid[y][x];
                    start->state = START;
                } else if (!end && &grid[y][x] != start) {
                    end = &grid[y][x];
                    end->state = END;
                }
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
            int x = sf::Mouse::getPosition(window).x / CELL_SIZE;
            int y = sf::Mouse::getPosition(window).y / CELL_SIZE;
            if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
                if (&grid[y][x] != start && &grid[y][x] != end) {
                    grid[y][x].state = WALL;
                }
            }
        }

        window.clear();
        for (auto& row : grid)
            for (auto& node : row)
                window.draw(node.getShape());
        window.display();
    }

    return 0;
}
