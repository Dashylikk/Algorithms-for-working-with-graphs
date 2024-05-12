#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <fstream>
#include <SFML/Graphics.hpp>

using namespace std;
const double M_PI = 3.14159265358979323846;



struct Edge {
    int to;
    int weight;
    Edge(int t, int w) : to(t), weight(w) {}
};

vector<vector<Edge>> readGraph(const string& filename, int& n, vector<pair<int, int>>& queries) {
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "Failed to open input file." << endl;
        exit(1);
    }

    infile >> n;
    int m;
    infile >> m;

    vector<vector<Edge>> graph(n + 1);

    for (int i = 0; i < m; ++i) {
        int u, v, w;
        infile >> u >> v >> w;
        graph[u].push_back({ v, w });
    }

    int numQueries;
    infile >> numQueries;
    queries.resize(numQueries);

    for (int i = 0; i < numQueries; ++i) {
        infile >> queries[i].first >> queries[i].second;
    }

    infile.close();

    return graph;
}



vector<int> bidijkstra(const vector<vector<Edge>>& graph, int n, int start) {
    vector<int> dist_start(n + 1, -1);
    vector<int> dist_end(n + 1, -1);
    vector<bool> visited_start(n + 1, false);
    vector<bool> visited_end(n + 1, false);
    dist_start[start] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq_start;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq_end;
    pq_start.push({ 0, start });

    while (!pq_start.empty() || !pq_end.empty()) {
        if (!pq_start.empty()) {
            int u_start = pq_start.top().second;
            pq_start.pop();

            if (visited_end[u_start]) break;
            visited_start[u_start] = true;

            for (const Edge& e : graph[u_start]) {
                int v = e.to;
                int weight = e.weight;

                if (dist_start[u_start] != -1 && (dist_start[u_start] + weight < dist_start[v] || dist_start[v] == -1)) {
                    dist_start[v] = dist_start[u_start] + weight;
                    pq_start.push({ dist_start[v], v });
                }
            }
        }

        if (!pq_end.empty()) {
            int u_end = pq_end.top().second;
            pq_end.pop();

            if (visited_start[u_end]) break;
            visited_end[u_end] = true;

            for (const Edge& e : graph[u_end]) {
                int v = e.to;
                int weight = e.weight;

                if (dist_end[u_end] != -1 && (dist_end[u_end] + weight < dist_end[v] || dist_end[v] == -1)) {
                    dist_end[v] = dist_end[u_end] + weight;
                    pq_end.push({ dist_end[v], v });
                }
            }
        }
    }

    vector<int> shortest_distances(n + 1);
    for (int i = 1; i <= n; ++i) {
        shortest_distances[i] = dist_start[i];
        if (dist_start[i] != -1 && dist_end[i] != -1) {
            shortest_distances[i] = min(dist_start[i], dist_end[i]);
        }
    }

    return shortest_distances;
}



void drawGraph(sf::RenderWindow& window, const vector<vector<Edge>>& graph) {
    const int radius = 20;
    const int centerX = 400;
    const int centerY = 300;
    const float angleStep = 2 * M_PI / graph.size();

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font." << endl;
        return;
    }

    for (size_t u = 1; u < graph.size(); ++u) {
        int x = centerX + static_cast<int>(200 * cos(angleStep * u));
        int y = centerY + static_cast<int>(200 * sin(angleStep * u));

        for (const Edge& e : graph[u]) {
            size_t v = e.to;

            int vx = centerX + static_cast<int>(200 * cos(angleStep * v));
            int vy = centerY + static_cast<int>(200 * sin(angleStep * v));

            sf::VertexArray line(sf::Lines, 2);
            line[0].position = sf::Vector2f(x, y);
            line[1].position = sf::Vector2f(vx, vy);
            line[0].color = sf::Color::Black;
            line[1].color = sf::Color::Black;
            window.draw(line);

            sf::Text weightText;
            weightText.setFont(font);
            weightText.setCharacterSize(15);
            weightText.setFillColor(sf::Color::Black);
            weightText.setString(to_string(e.weight));

            float textX = (x + vx) / 2.0f;
            float textY = (y + vy) / 2.0f;
            sf::Vector2f delta = sf::Vector2f(vx - x, vy - y);
            float length = sqrt(delta.x * delta.x + delta.y * delta.y);
            delta /= length;
            textX -= delta.x * 10;
            textY -= delta.y * 10;

            weightText.setPosition(textX, textY);
            window.draw(weightText);
        }
    }

    // Draw vertices
    for (size_t u = 1; u < graph.size(); ++u) {
        int x = centerX + static_cast<int>(200 * cos(angleStep * u));
        int y = centerY + static_cast<int>(200 * sin(angleStep * u));

        sf::CircleShape circle(radius);
        circle.setFillColor(sf::Color::Blue);
        circle.setPosition(x - radius, y - radius);
        window.draw(circle);

        sf::Text text;
        text.setFont(font);
        text.setCharacterSize(15);
        text.setFillColor(sf::Color::White);
        text.setString(to_string(u));
        sf::FloatRect textBounds = text.getLocalBounds();
        text.setOrigin(textBounds.left + textBounds.width / 2.0f, textBounds.top + textBounds.height / 2.0f);
        text.setPosition(x, y);
        window.draw(text);
    }
}



int main() {
    string filename = "inputt.txt";
    int n;
    vector<pair<int, int>> queries;
    vector<vector<Edge>> graph = readGraph(filename, n, queries);

    sf::RenderWindow window(sf::VideoMode(800, 600), "Social Network Graph");

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font." << endl;
        return 1;
    }

    sf::Text resultText;
    resultText.setFont(font);
    resultText.setCharacterSize(20);
    resultText.setFillColor(sf::Color::Black);
    resultText.setPosition(10, 10);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color::White);

        drawGraph(window, graph);

        string resultString;
        for (const auto& query : queries) {
            int source = query.first;
            int dest = query.second;

            vector<int> distFromSource = bidijkstra(graph, n, source);
            int shortestDistance = distFromSource[dest];

            resultString += "Output: " + to_string(shortestDistance) + "\n";
        }

        resultText.setString(resultString);

        window.draw(resultText);

        window.display();
    }

    return 0;
}
