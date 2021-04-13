#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

//only black and while

std::vector<std::vector<uint16_t>> getImage(std::string fileName) {
    auto im = cv::imread(fileName, cv::IMREAD_GRAYSCALE);
    std::cerr << "rows: " << im.rows << "\ncols: " << im.cols << "\nchannels" << im.channels() << std::endl;
    size_t sz = std::max(im.rows, im.cols);
    std::vector<std::vector<uint16_t>> res(sz, std::vector<uint16_t>(sz, 0));
    uint64_t sum = 0;
    for (size_t i = 0; i < im.rows; ++i) {
        for (size_t j = 0; j < im.cols; ++j) {
            auto color = (static_cast<uint32_t>(im.at<uchar>(i, j)));
            assert(color == 0 || color == 255);
            res[i][j] = (color == 0);
            sum += res[i][j];
        }
    }
    std::cerr << "black count:" << sum << std::endl;
    return res;

}


struct Point {
    int32_t x, y;
};

bool operator<(const Point& lhs, const Point& rhs) {
    return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
}

bool isValid(Point pt, int32_t sz) {
    if (pt.x < 0) {
        return false;
    } else if(pt.y < 0) {
        return false;
    } else if(pt.x >= sz) {
        return false;
    } else if(pt.y >= sz) {
        return false;
    }
    return true;
}

int distance(Point lhs, Point rhs) {
    return std::min(abs(lhs.x - rhs.x), abs(lhs.y - rhs.y));
}
using Matrix = std::vector<std::vector<uint16_t>>;
std::vector<std::set<Point>> Partition(queue<Point>& qu, int32_t eps, const Matrix& matrix) {
    std::vector<std::set<Point>> res;
    std::set<Point> pts;
    while(!qu.empty()) {
        pts.insert(qu.front());
        qu.pop();
    }
    while(!pts.empty()) {
        std::set<Point> neighbours;
        Point pt = *pts.begin();
        pts.erase(pts.begin());
        queue<Point> q;
        q.push(pt);
        neighbours.insert(pt);
        while(!q.empty()) {
            auto p = q.front();
            q.pop();
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    Point next = {p.x + dx, p.y + dy};
                    if (!isValid(next, matrix.size())) {
                        continue;
                    }
                    if (!pts.count(next)) {
                        continue;
                    }
                    if (neighbours.count(next)) {
                        continue;
                    }
                    if (!matrix[next.x][next.y]) {
                        continue;
                    }
                    /*if (distance(pt, next) > eps) {
                        continue
                    }*/
                    neighbours.insert(next);
                    q.push(next);
                    pts.erase(next);
                }
            }

        }

        res.push_back(neighbours);
    }
    return res;


}
std::vector<std::set<Point>> getDirections(std::vector<std::vector<uint16_t>>& matrix, Point pt, uint32_t line_width) {
    uint32_t step_count = line_width * 20;
    queue<Point> q;
    q.push(pt);
    std::set<Point> used;
    used.insert(pt);
    for (uint32_t step = 0; step < step_count; ++step) {
        queue<Point> newq;
        while(!q.empty()) {
            auto p = q.front();
            q.pop();
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    Point next = {p.x + dx, p.y + dy};
                    if (!isValid(next, matrix.size())) {
                        continue;
                    }
                    if (used.count(next)) {
                        continue;
                    }
                    if (!matrix[next.x][next.y]) {
                        continue;
                    }
                    used.insert(next);
                    newq.push(next);
                }
            }
        }
        swap(q, newq);
    }
    int32_t eps = line_width * 3;
    return Partition(q, eps, matrix);
}

bool isEq(double lhs, double rhs) {
    return (fabs(lhs - rhs) < 0.001);
}
bool checkLine(Point lpt, Point rpt, Point pt) {
    if (lpt.x == rpt.x) {
        return pt.x == lpt.x;
    }
    int dx = rpt.x - lpt.x;
    int dy = rpt.y - lpt.y;
    int d_x = pt.x - lpt.x;
    int d_y = pt.y - lpt.y;
    return isEq(1. * dy / dx, 1. * d_y / d_x);

}
bool checkLine(const set<Point>& l, const set<Point>& r, Point pt) {
    for (auto lpt : l) {
        for (auto rpt : r) {
            if (checkLine(lpt, rpt, pt)) {
                return true;
            }
        }
    }
    return false;
}
void recolor(Matrix& matrix, Point pt, int32_t eps) {
    for (int dx = -eps; dx <= eps; ++dx) {
        for (int dy = -eps; dy <= eps; ++dy) {
            Point next = {pt.x + dx, pt.y + dy};
            if(isValid(next, matrix.size())) {
                if (matrix[next.x][next.y]) {
                    matrix[next.x][next.y] = 2;
                }
            }
        }
    }
}

bool checkPoint(std::vector<std::vector<uint16_t>>& matrix, Point pt, uint32_t line_width) {
    uint32_t epsilon = line_width * 3;
    std::vector<std::set<Point>> directions = getDirections(matrix, pt, line_width);
    if (directions.size() <= 2) {
        return false;
    }
    for (size_t i = 0; i < directions.size(); ++i) {
        for (size_t j = i + 1; j < directions.size(); ++j) {
            if (checkLine(directions[i], directions[j], pt)) {
                recolor(matrix, pt, line_width * 50);
                return true;
            }
        }
    }
    return false;
}
uint32_t solve(std::string fileName, uint32_t line_width = 3) {
    auto matrix = getImage(fileName);
    auto sz = matrix.size();
    uint32_t res = 0;
    for (size_t i = 0; i < sz; ++i) {
        for (size_t j = 0; j < sz; ++j) {
            if (matrix[i][j] == 1) {
                res += checkPoint(matrix, {(int)i, (int)j}, line_width);
            }
        }
    }
    return res;
}

int main(int argc, char** argv) {
    std::string fileName = "test.png";
    std::cout << solve(fileName);
}
