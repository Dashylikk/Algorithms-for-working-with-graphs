#include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include <sstream>
#include <cmath>
#include <functional>
#include <stack>
#include <climits>

const double PI = 3.14159265358979323846;
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 1000;
const int RADIUS = 300;
const int CIRCLE_RADIUS = 20;
const sf::Color NODE_COLOR = sf::Color::Blue;
const sf::Color PATH_COLOR = sf::Color::Red;

class Button {
public:
    Button(sf::Texture* normal, sf::Texture* clicked, std::string text, sf::Vector2f location);
    void checkClick(sf::Vector2f mousePos);
    void setState(bool);
    bool getState();
    sf::Sprite* getSprite();
    sf::Text* getText();
    void setAction(std::function<void()> action);
    void draw(sf::RenderWindow& window);
    void addButton(Button* button);
    void setParent(Button* parent);
private:
    sf::Sprite normalSprite;
    sf::Sprite clickedSprite;
    sf::Sprite* currentSprite;
    sf::Text buttonText;
    bool currentState;
    std::function<void()> buttonAction;
    std::vector<Button*> subButtons;
    Button* parentButton;
};

Button::Button(sf::Texture* normal, sf::Texture* clicked, std::string text, sf::Vector2f location) {
    normalSprite.setTexture(*normal);
    clickedSprite.setTexture(*clicked);
    currentSprite = &normalSprite;
    currentState = false;

    buttonText.setString(text);
    buttonText.setCharacterSize(18);
    buttonText.setFillColor(sf::Color::Black);
    buttonText.setPosition(location.x + (normalSprite.getLocalBounds().width - buttonText.getLocalBounds().width) / 2, location.y + (normalSprite.getLocalBounds().height - buttonText.getLocalBounds().height) / 2);

    normalSprite.setPosition(location);
    clickedSprite.setPosition(location);

    sf::Vector2f newSize(200, 80);
    normalSprite.setScale(newSize.x / normalSprite.getLocalBounds().width, newSize.y / normalSprite.getLocalBounds().height);
    clickedSprite.setScale(newSize.x / clickedSprite.getLocalBounds().width, newSize.y / clickedSprite.getLocalBounds().height);

    buttonAction = nullptr;
    parentButton = nullptr;
}

void Button::checkClick(sf::Vector2f mousePos) {
    if (currentSprite->getGlobalBounds().contains(mousePos)) {
        currentState = !currentState;
        currentSprite = currentState ? &clickedSprite : &normalSprite;

        if (buttonAction) {
            buttonAction();
        }
    }

    if (currentState) {
        for (Button* button : subButtons) {
            button->checkClick(mousePos);
        }
    }
}

void Button::setState(bool state) {
    currentState = state;
    currentSprite = currentState ? &clickedSprite : &normalSprite;
}

bool Button::getState() {
    return currentState;
}

sf::Sprite* Button::getSprite() {
    return currentSprite;
}

sf::Text* Button::getText() {
    return &buttonText;
}

void Button::setAction(std::function<void()> action) {
    buttonAction = action;
}

void Button::draw(sf::RenderWindow& window) {
    window.draw(*currentSprite);
    window.draw(buttonText);

    if (currentState) {
        for (Button* button : subButtons) {
            button->draw(window);
        }
    }
}

void Button::addButton(Button* button) {
    button->setParent(this);
    subButtons.push_back(button);
}

void Button::setParent(Button* parent) {
    parentButton = parent;
}
class GraphNode {
public:
    int x, y;
    std::vector<int> neighbors;
    std::vector<int> weights;
    bool visited;
    double f;

    bool isNeighbor(int v) const {
        for (int neighbor : neighbors) {
            if (neighbor == v) {
                return true;
            }
        }
        return false;
    }

    int getWeight(int v) const {
        for (size_t i = 0; i < neighbors.size(); ++i) {
            if (neighbors[i] == v) {
                return weights[i];
            }
        }
    }
};

void drawNode(sf::RenderWindow& window, int x, int y, sf::Color color, int nodeNumber) {
    sf::CircleShape circle(CIRCLE_RADIUS);
    circle.setFillColor(color);
    circle.setPosition(x - CIRCLE_RADIUS, y - CIRCLE_RADIUS);
    window.draw(circle);

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font." << std::endl;
        return;
    }
    sf::Text text;
    text.setFont(font);
    text.setString(std::to_string(nodeNumber));
    text.setCharacterSize(18);
    text.setFillColor(sf::Color::White);
    text.setStyle(sf::Text::Bold);

    sf::FloatRect textBounds = text.getLocalBounds();
    text.setOrigin(textBounds.left + textBounds.width / 2, textBounds.top + textBounds.height / 2);
    text.setPosition(x, y);

    window.draw(text);
}


void drawLine(sf::RenderWindow& window, int x1, int y1, int x2, int y2, sf::Color color, float thickness) {
    sf::Vector2f direction(x2 - x1, y2 - y1);
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    direction /= length;

    sf::Vector2f normal(-direction.y, direction.x);

    sf::VertexArray line(sf::Lines, 4);
    line[0].position = sf::Vector2f(x1 + normal.x * thickness / 2, y1 + normal.y * thickness / 2);
    line[1].position = sf::Vector2f(x2 + normal.x * thickness / 2, y2 + normal.y * thickness / 2);
    line[2].position = sf::Vector2f(x2 - normal.x * thickness / 2, y2 - normal.y * thickness / 2);
    line[3].position = sf::Vector2f(x1 - normal.x * thickness / 2, y1 - normal.y * thickness / 2);

    for (int i = 0; i < 4; ++i) {
        line[i].color = color;
    }

    window.draw(line);
}


void resetVisited(std::vector<GraphNode>& graph) {
    for (auto& node : graph) {
        node.visited = false;
    }
}

void visualizeBFS(sf::RenderWindow& window, std::vector<GraphNode>& graph, int start, int end) {
    std::queue<int> q;
    q.push(start);
    graph[start].visited = true;

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        if (current == end)
            break;

        for (int neighbor : graph[current].neighbors) {
            if (!graph[neighbor].visited) {
                q.push(neighbor);
                graph[neighbor].visited = true;
                drawLine(window, graph[current].x, graph[current].y, graph[neighbor].x, graph[neighbor].y, PATH_COLOR, 3.0f);
            }
        }
    }
}

void dfs(std::vector<GraphNode>& graph, int current, int target, sf::RenderWindow& window) {
    graph[current].visited = true;

    if (current == target)
        return;

    for (int neighbor : graph[current].neighbors) {
        if (!graph[neighbor].visited) {
            drawLine(window, graph[current].x, graph[current].y, graph[neighbor].x, graph[neighbor].y, PATH_COLOR, 3.0f);
            dfs(graph, neighbor, target, window);
        }
    }
}

void visualizePrim(sf::RenderWindow& window, std::vector<GraphNode>& graph) {
    std::vector<bool> inMST(graph.size(), false);
    std::vector<int> parent(graph.size(), -1);
    std::vector<int> key(graph.size(), INT_MAX);

    key[0] = 0;

    for (int count = 0; count < graph.size() - 1; ++count) {
        int u = -1;

        for (int v = 0; v < graph.size(); ++v) {
            if (!inMST[v] && (u == -1 || key[v] < key[u])) {
                u = v;
            }
        }

        inMST[u] = true;

        for (int v = 0; v < graph.size(); ++v) {
            if (graph[u].isNeighbor(v) && !inMST[v] && graph[u].getWeight(v) < key[v]) {
                parent[v] = u;
                key[v] = graph[u].getWeight(v);
            }
        }
    }

    for (int v = 1; v < graph.size(); ++v) {
        drawLine(window, graph[parent[v]].x, graph[parent[v]].y, graph[v].x, graph[v].y, PATH_COLOR, 3.0f);
    }
}

void visualizeDijkstra(sf::RenderWindow& window, std::vector<GraphNode>& graph, int start, int end) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
    std::vector<int> dist(graph.size(), INT_MAX);
    std::vector<int> parent(graph.size(), -1);

    pq.push(std::make_pair(0, start));
    dist[start] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (int v : graph[u].neighbors) {
            int weight = graph[u].getWeight(v);
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push(std::make_pair(dist[v], v));
            }
        }
    }

    int current = end;
    while (current != -1) {
        int prev = parent[current];
        if (prev != -1) {
            drawLine(window, graph[current].x, graph[current].y, graph[prev].x, graph[prev].y, PATH_COLOR, 3.0f);
        }
        current = prev;
    }
}
struct AStarNode {
    int index;
    double g;
    double f;

    AStarNode(int idx, double gVal, double fVal) : index(idx), g(gVal), f(fVal) {}

    bool operator<(const AStarNode& other) const {
        return f > other.f;
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void visualizeAStar(sf::RenderWindow& window, std::vector<GraphNode>& graph, int start, int end) {
    std::priority_queue<AStarNode> openSet;
    openSet.push(AStarNode(start, 0, heuristic(graph[start].x, graph[start].y, graph[end].x, graph[end].y)));

    std::vector<double> gValues(graph.size(), std::numeric_limits<double>::infinity());
    gValues[start] = 0;

    std::vector<int> cameFrom(graph.size(), -1);

    while (!openSet.empty()) {
        int current = openSet.top().index;
        openSet.pop();

        if (current == end) {
            std::stack<int> path;
            while (current != -1) {
                path.push(current);
                current = cameFrom[current];
            }

            int prevNode = path.top();
            path.pop();
            while (!path.empty()) {
                int nextNode = path.top();
                path.pop();
                drawLine(window, graph[prevNode].x, graph[prevNode].y, graph[nextNode].x, graph[nextNode].y, PATH_COLOR, 3.0f);
                prevNode = nextNode;
            }
            return;
        }

        for (int neighbor : graph[current].neighbors) {
            double tentative_g = gValues[current] + graph[current].getWeight(neighbor);
            if (tentative_g < gValues[neighbor]) {
                cameFrom[neighbor] = current;
                gValues[neighbor] = tentative_g;
                double fValue = tentative_g + heuristic(graph[neighbor].x, graph[neighbor].y, graph[end].x, graph[end].y);
                openSet.push(AStarNode(neighbor, tentative_g, fValue));
            }
        }
    }
}

int fordFulkerson(std::vector<std::vector<int>>& residualGraph, int source, int sink) {
    int maxFlow = 0;

    std::vector<int> parent(residualGraph.size(), -1);

    while (true) {
        std::vector<bool> visited(residualGraph.size(), false);
        std::queue<int> q;
        q.push(source);
        visited[source] = true;
        parent[source] = -1;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (int v = 0; v < residualGraph.size(); ++v) {
                if (!visited[v] && residualGraph[u][v] > 0) {
                    q.push(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }

        if (!visited[sink]) {
            break;
        }

        int pathFlow = INT_MAX;

        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = std::min(pathFlow, residualGraph[u][v]);
        }

        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residualGraph[u][v] -= pathFlow;
            residualGraph[v][u] += pathFlow;
        }

        maxFlow += pathFlow;
    }

    return maxFlow;
}


std::vector<GraphNode> constructGraph(int numNodes, const std::vector<std::tuple<int, int, int>>& edges) {
    std::vector<GraphNode> graph(numNodes);
    int centerX = WINDOW_WIDTH / 2;
    int centerY = WINDOW_HEIGHT / 2;

    graph[numNodes - 1].x = centerX;
    graph[numNodes - 1].y = centerY;

    double angleIncrement = 2 * PI / (numNodes - 1);
    double currentAngle = 0;

    for (int i = 0; i < numNodes - 1; ++i) {
        graph[i].x = centerX + RADIUS * std::cos(currentAngle);
        graph[i].y = centerY + RADIUS * std::sin(currentAngle);
        currentAngle += angleIncrement;
    }

    for (const auto& edge : edges) {
        int u, v, weight;
        std::tie(u, v, weight) = edge;
        graph[u].neighbors.push_back(v);
        graph[v].neighbors.push_back(u);
        graph[u].weights.push_back(weight);
        graph[v].weights.push_back(weight);
    }

    return graph;
}


std::vector<std::tuple<int, int, int>> readGraphFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::tuple<int, int, int>> edges;
    if (file.is_open()) {
        int u, v, weight;
        while (file >> u >> v >> weight) {
            edges.push_back({ u, v, weight });
        }
        file.close();
    }
    return edges;
}


int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Algorithms");
    window.setFramerateLimit(60);
    sf::Texture normalTexture, clickedTexture;
    if (!normalTexture.loadFromFile("normal_button.png") || !clickedTexture.loadFromFile("clicked_button.png")) {
        std::cerr << "Failed to load button textures." << std::endl;
        return 1;
    }

    std::vector<std::tuple<int, int, int>> edges = readGraphFromFile("graph.txt");

    int numNodes = 0;
    for (const auto& edge : edges) {
        numNodes = std::max(numNodes, std::max(std::get<0>(edge), std::get<1>(edge)) + 1);
    }

    std::vector<GraphNode> graph = constructGraph(numNodes, edges);

    window.clear(sf::Color::White);

    Button mainButton1(&normalTexture, &clickedTexture, "Main Button 1", sf::Vector2f(100, 400));
    Button mainButton2(&normalTexture, &clickedTexture, "Main Button 2", sf::Vector2f(300, 400));
    Button mainButton3(&normalTexture, &clickedTexture, "Main Button 3", sf::Vector2f(500, 400));
    Button mainButton4(&normalTexture, &clickedTexture, "Main Button 4", sf::Vector2f(700, 400));
    Button mainButton5(&normalTexture, &clickedTexture, "Main Button 5", sf::Vector2f(900, 400));
    Button mainButton6(&normalTexture, &clickedTexture, "Main Button 6", sf::Vector2f(500, 600));


    mainButton1.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow bfsWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "BFS");
        bfsWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(bfsWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }

        resetVisited(graph);
        visualizeBFS(bfsWindow, graph, 0, numNodes - 1);

        for (const auto& node : graph) {
            drawNode(bfsWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }
        bfsWindow.display();

        while (bfsWindow.isOpen()) {
            sf::Event bfsEvent;

            while (bfsWindow.pollEvent(bfsEvent)) {
                if (bfsEvent.type == sf::Event::Closed) {
                    bfsWindow.close();
                }
            }
        }
        });
    mainButton2.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow dfsWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "DFS");
        dfsWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(dfsWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }
        resetVisited(graph);
        dfs(graph, 0, numNodes - 1, dfsWindow);

        for (const auto& node : graph) {
            drawNode(dfsWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }

        dfsWindow.display();

        while (dfsWindow.isOpen()) {
            sf::Event dfsEvent;

            while (dfsWindow.pollEvent(dfsEvent)) {
                if (dfsEvent.type == sf::Event::Closed) {
                    dfsWindow.close();
                }
            }
        }
        });

    mainButton3.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow primWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Prim's Algorithm");
        primWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(primWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }
        resetVisited(graph);
        visualizePrim(primWindow, graph);

        for (const auto& node : graph) {
            drawNode(primWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }

        primWindow.display();

        while (primWindow.isOpen()) {
            sf::Event primEvent;

            while (primWindow.pollEvent(primEvent)) {
                if (primEvent.type == sf::Event::Closed) {
                    primWindow.close();
                }
            }
        }
        });
    mainButton4.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow dijkstraWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Dijkstra's Algorithm");
        dijkstraWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(dijkstraWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }
        resetVisited(graph);
        visualizeDijkstra(dijkstraWindow, graph, 0, numNodes - 1);

        for (const auto& node : graph) {
            drawNode(dijkstraWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }

        dijkstraWindow.display();

        while (dijkstraWindow.isOpen()) {
            sf::Event dijkstraEvent;

            while (dijkstraWindow.pollEvent(dijkstraEvent)) {
                if (dijkstraEvent.type == sf::Event::Closed) {
                    dijkstraWindow.close();
                }
            }
        }
        });
    mainButton5.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow aStarWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "A* Algorithm");
        aStarWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(aStarWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }
        resetVisited(graph);
        visualizeAStar(aStarWindow, graph, 0, numNodes - 1);

        for (const auto& node : graph) {
            drawNode(aStarWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }

        aStarWindow.display();

        while (aStarWindow.isOpen()) {
            sf::Event aStarEvent;

            while (aStarWindow.pollEvent(aStarEvent)) {
                if (aStarEvent.type == sf::Event::Closed) {
                    aStarWindow.close();
                }
            }
        }
        });

    mainButton6.setAction([&]() {
        int nodeCounter = 0;
        sf::RenderWindow fordFulkersonWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Ford-Fulkerson Algorithm");
        fordFulkersonWindow.clear(sf::Color::White);

        for (const auto& edge : edges) {
            drawLine(fordFulkersonWindow, graph[std::get<0>(edge)].x, graph[std::get<0>(edge)].y,
                graph[std::get<1>(edge)].x, graph[std::get<1>(edge)].y, sf::Color::Black, 3.0f);
        }

        std::vector<std::vector<int>> residualGraph(numNodes, std::vector<int>(numNodes, 0));
        for (int u = 0; u < numNodes; ++u) {
            for (size_t i = 0; i < graph[u].neighbors.size(); ++i) {
                int v = graph[u].neighbors[i];
                int weight = graph[u].weights[i];
                residualGraph[u][v] = weight;
            }
        }
        for (const auto& node : graph) {
            drawNode(fordFulkersonWindow, node.x, node.y, NODE_COLOR, nodeCounter++);
        }
        int maxFlow = fordFulkerson(residualGraph, 0, numNodes - 1);

        sf::Font font;
        if (!font.loadFromFile("arial.ttf")) {
            std::cerr << "Failed to load font." << std::endl;
            return;
        }

        sf::Text text;
        text.setFont(font);
        text.setString("Max flow: " + std::to_string(maxFlow));
        text.setCharacterSize(24);
        text.setFillColor(sf::Color::Black);
        text.setPosition(10, 10);

        fordFulkersonWindow.draw(text);

        fordFulkersonWindow.display();

        while (fordFulkersonWindow.isOpen()) {
            sf::Event fordFulkersonEvent;

            while (fordFulkersonWindow.pollEvent(fordFulkersonEvent)) {
                if (fordFulkersonEvent.type == sf::Event::Closed) {
                    fordFulkersonWindow.close();
                }
            }
        }
        });


    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font." << std::endl;
        return 1;
    }

    sf::Text buttonText1, buttonText2, buttonText3, buttonText4, buttonText5, buttonText6, buttonText7;
    buttonText1.setFont(font);
    buttonText1.setString("BFS");
    buttonText1.setCharacterSize(24);
    buttonText1.setFillColor(sf::Color::Black);
    buttonText1.setPosition(175, 360);

    buttonText2.setFont(font);
    buttonText2.setString("DFS");
    buttonText2.setCharacterSize(24);
    buttonText2.setFillColor(sf::Color::Black);
    buttonText2.setPosition(375, 360);

    buttonText3.setFont(font);
    buttonText3.setString("Prim's Algorithm");
    buttonText3.setCharacterSize(24);
    buttonText3.setFillColor(sf::Color::Black);
    buttonText3.setPosition(510, 360);

    buttonText4.setFont(font);
    buttonText4.setString("Dijkstra's Algorithm");
    buttonText4.setCharacterSize(22);
    buttonText4.setFillColor(sf::Color::Black);
    buttonText4.setPosition(705, 360);

    buttonText5.setFont(font);
    buttonText5.setString("A* Algorithm");
    buttonText5.setCharacterSize(24);
    buttonText5.setFillColor(sf::Color::Black);
    buttonText5.setPosition(930, 360);

    buttonText6.setFont(font);
    buttonText6.setString("Ford-Fulkerson Algorithm");
    buttonText6.setCharacterSize(24);
    buttonText6.setFillColor(sf::Color::Black);
    buttonText6.setPosition(480, 560);


    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f mousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
                    mainButton1.checkClick(mousePos);
                    mainButton2.checkClick(mousePos);
                    mainButton3.checkClick(mousePos);
                    mainButton4.checkClick(mousePos);
                    mainButton5.checkClick(mousePos);
                    mainButton6.checkClick(mousePos);
                }
            }
        }

        window.clear(sf::Color::White);
        mainButton1.draw(window);
        mainButton2.draw(window);
        mainButton3.draw(window);
        mainButton4.draw(window);
        mainButton5.draw(window);
        mainButton6.draw(window);
        window.draw(buttonText1);
        window.draw(buttonText2);
        window.draw(buttonText3);
        window.draw(buttonText4);
        window.draw(buttonText5);
        window.draw(buttonText6);
        window.draw(buttonText7);
        window.display();
    }

    return 0;
}
