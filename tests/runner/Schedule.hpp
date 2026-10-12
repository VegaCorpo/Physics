#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace qa {

    struct Segment {
            double dt = 0.0;
            std::size_t frames = 0;
    };

    class Schedule {
        public:
            Schedule(double dt, std::size_t frames) { this->_append({dt, frames}); }

            [[nodiscard]] static Schedule parse(const std::string& specification)
            {
                Schedule schedule;
                std::size_t start = 0;

                while (start < specification.size()) {
                    const std::size_t end = std::min(specification.find(',', start), specification.size());
                    schedule._append(_parseSegment(specification.substr(start, end - start)));
                    start = end + 1;
                }
                if (schedule._segments.empty())
                    throw std::invalid_argument("empty dt schedule");
                return schedule;
            }

            [[nodiscard]] std::size_t totalFrames() const { return this->_lastFrames.back(); }

            [[nodiscard]] double dtAt(std::size_t frame) const
            {
                const auto found = std::ranges::lower_bound(this->_lastFrames, frame);
                const auto index =
                    std::min(static_cast<std::size_t>(found - this->_lastFrames.begin()), this->_segments.size() - 1);

                return this->_segments[index].dt;
            }

            [[nodiscard]] double elapsedAt(std::size_t frame) const
            {
                double elapsed = 0.0;
                std::size_t remaining = frame;

                for (const Segment& segment : this->_segments) {
                    const std::size_t taken = std::min(remaining, segment.frames);
                    elapsed += segment.dt * static_cast<double>(taken);
                    remaining -= taken;
                }
                return elapsed;
            }

        private:
            Schedule() = default;

            void _append(const Segment& segment)
            {
                const std::size_t previous = this->_lastFrames.empty() ? 0 : this->_lastFrames.back();

                this->_segments.push_back(segment);
                this->_lastFrames.push_back(previous + segment.frames);
            }

            [[nodiscard]] static Segment _parseSegment(const std::string& text)
            {
                const std::size_t separator = text.find(':');

                if (separator == std::string::npos)
                    throw std::invalid_argument("dt schedule segment '" + text + "' is not DT:FRAMES");
                return {std::stod(text.substr(0, separator)), std::stoul(text.substr(separator + 1))};
            }

            std::vector<Segment> _segments;
            std::vector<std::size_t> _lastFrames;
    };

} // namespace qa
