#pragma once

#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <interfaces/IPhysicsEngine.hpp>

namespace qa {

    /**
     * @brief Loads a physics module shared library and exposes its factory.
     *
     * Only the public `get_engine` symbol is used, exactly like Core does, so
     * any library implementing common::IPhysicsEngine can be driven.
     * The library stays mapped for the lifetime of this object: every engine
     * created from it must be destroyed before the Plugin.
     */
    class Plugin {
        public:
            explicit Plugin(const std::string& path)
            {
                this->_handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
                if (this->_handle == nullptr)
                    throw std::runtime_error(std::string("dlopen failed: ") + dlerror());

                void* symbol = dlsym(this->_handle, "get_engine");
                if (symbol == nullptr) {
                    std::string message = std::string("dlsym(get_engine) failed: ") + dlerror();
                    dlclose(this->_handle);
                    throw std::runtime_error(message);
                }
                this->_factory = reinterpret_cast<Factory>(symbol);
            }

            Plugin(const Plugin&) = delete;
            Plugin& operator=(const Plugin&) = delete;

            /**
             * The library is deliberately never dlclose()d: the module keeps a
             * TBB worker pool alive after the engine is destroyed, and unmapping
             * the code those threads run crashes the process. The handle is
             * released by process exit instead.
             */
            ~Plugin() = default;

            [[nodiscard]] std::unique_ptr<common::IPhysicsEngine> create() const { return this->_factory(); }

        private:
            using Factory = std::unique_ptr<common::IPhysicsEngine> (*)();

            void* _handle = nullptr;
            Factory _factory = nullptr;
    };

} // namespace qa
