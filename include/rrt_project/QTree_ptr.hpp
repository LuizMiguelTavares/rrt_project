
    template<typename Node>
    class QuadTree2 {
    public:
        QuadTree2(const Rectangle& boundary, std::shared_ptr<Node> start, const int capacity = DEFAULT_CAPACITY)
                : boundary(boundary), capacity(capacity), divided(false), depth(0), parent(nullptr), position("root") {
                    if (capacity < 1) {
                        throw std::range_error("capacity must be greater than 0");
                    }

                    this->nodes.reserve(5000);
                    this->insert(start);
                }

        static std::unique_ptr<QuadTree2> Create(const Rectangle& boundary, const int capacity, const int depth, const QuadTree2* parent, const std::string& position) {
            return std::unique_ptr<QuadTree2>(new QuadTree2(boundary, capacity, depth, parent, position));
        }

        bool insert(std::shared_ptr<Node> node) {
            Point point(node->x, node->y);

            if (!boundary.contains(point)) {
                assert(parent != nullptr && "Node is out of bounds of the QuadTree2");
                return false;
            }

            if (this->nodes.size() < capacity || depth == MAX_DEPTH) {
                this->nodes.push_back(node);
                return true;
            }

            if (!divided) {
                subdivide();
            }

            if (northeast->insert(node) || northwest->insert(node) || 
                southeast->insert(node) || southwest->insert(node)) {
                return true;
            }

            // This should never happen
            return false;
        }

        std::shared_ptr<Node> nearest_neighbor(const std::shared_ptr<Node>& node) {
            // Assert that the node is within the boundary of the QuadTree2
            assert(this->boundary.contains(Point(node->x, node->y)) && "Can't find nearest neighbor for a node outside the boundary of the QuadTree2");
            
            Point point(node->x, node->y);
            QuadTree2* current_quadtree2 = find_quadtree2(this, point);

            std::shared_ptr<Node> current_closest_node = nullptr;
            float distance = find_parent_distance(current_quadtree2, point, current_closest_node);

            std::vector<std::shared_ptr<Node>> nodes;

            std::shared_ptr<Node> possible_nearest_node;

            if(distance == 0){
                nodes = this->nodes;
                possible_nearest_node = nodes[0];
            } else {
                possible_nearest_node = current_closest_node;
                Rectangle range = Rectangle(point.x, point.y, std::sqrt(distance)*2, std::sqrt(distance)*2);
                query(range, nodes);
            }

            std::shared_ptr<Node> nearest_node  = possible_nearest_node;

            // Iterate over the nodes and find the nearest one to the given point
            for (const auto& p : nodes) {
                Point p_point(p->x, p->y);
                if (p_point.sqDistanceFrom(point) <= distance) {
                    distance = p_point.sqDistanceFrom(point);
                    nearest_node = p;
                }
            }
            return nearest_node;
        }

        void query(const Rectangle& range, std::vector<std::shared_ptr<Node>>& found) const {
            if (!boundary.intersects(range)) {
                return;
            }

            for (const auto& node : this->nodes) {
                Point point(node->x, node->y);
                if (range.contains(point)) {
                    found.push_back(node);
                }
            }

            if (divided) {
                this->northeast->query(range, found);
                this->northwest->query(range, found);
                this->southeast->query(range, found);
                this->southwest->query(range, found);
            }

            return;
        }

        static const int DEFAULT_CAPACITY = 4;
        static const int MAX_DEPTH = 8;
        const Rectangle boundary;
        const int capacity;
        std::vector<std::shared_ptr<Node>> nodes;
        bool divided;
        std::unique_ptr<QuadTree2> northeast, northwest, southeast, southwest;
        const QuadTree2 *parent;
        const std::string position;
        const int depth;

    private:
        QuadTree2(const Rectangle& boundary, const int capacity, const int depth, const QuadTree2 *parent, const std::string& position)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth), parent(parent), position(position) {
                this->nodes.reserve(5000);
            }

        void subdivide() {
            float x = boundary.x;
            float y = boundary.y;
            float w = boundary.w / 2.0;
            float h = boundary.h / 2.0;

            Rectangle neRect(x + w / 2, y - h / 2, w, h);
            Rectangle nwRect(x - w / 2, y - h / 2, w, h);
            Rectangle seRect(x + w / 2, y + h / 2, w, h);
            Rectangle swRect(x - w / 2, y + h / 2, w, h);

            this->northeast = QuadTree2::Create(neRect, capacity, depth + 1, this, "northeast");
            this->northwest = QuadTree2::Create(nwRect, capacity, depth + 1, this, "northwest");
            this->southeast = QuadTree2::Create(seRect, capacity, depth + 1, this, "southeast");
            this->southwest = QuadTree2::Create(swRect, capacity, depth + 1, this, "southwest");

            divided = true;
        }

        QuadTree2* find_quadtree2(QuadTree2* quad_tree, const Point& point) const {
            if (quad_tree->divided) {
                if (quad_tree->northeast->boundary.contains(point)) {
                    return find_quadtree2(quad_tree->northeast.get(), point);
                } else if (quad_tree->northwest->boundary.contains(point)) {
                    return find_quadtree2(quad_tree->northwest.get(), point);
                } else if (quad_tree->southeast->boundary.contains(point)) {
                    return find_quadtree2(quad_tree->southeast.get(), point);
                } else if (quad_tree->southwest->boundary.contains(point)) {
                    return find_quadtree2(quad_tree->southwest.get(), point);
                }
            }
            return quad_tree;
        }

        float find_parent_distance(const QuadTree2* quadtree2, const Point& point, std::shared_ptr<Node>& current_closest_node) const {
            if (quadtree2->parent == nullptr) {
                return 0; // Code to calculate all the distances because there are not much points
                        // Mostly shouldn't happen
            }

            if (quadtree2->nodes.size() > 0) {
                current_closest_node = quadtree2->nodes[0];
                Point p(quadtree2->nodes[0]->x, quadtree2->nodes[0]->y);
                return p.sqDistanceFrom(point);
            } else {
                return recursive_search_for_distance(quadtree2, point, current_closest_node);
            }
        }

        float recursive_search_for_distance(const QuadTree2* quadtree2, const Point& point, std::shared_ptr<Node>& current_closest_node) const {
            // Add logic to handle position-based node search from parent
            return 0; // Placeholder to maintain structure; implement based on specific criteria
        }
    };