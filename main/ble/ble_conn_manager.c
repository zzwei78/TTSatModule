/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "ble_conn_manager.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BLE_CONN_MGR";

// ============================================================
// Global State
// ============================================================

static ble_conn_state_t g_connections[BLE_MAX_CONNECTIONS];
static int g_connection_count = 0;
static bool g_initialized = false;

// ============================================================
// Helper Functions
// ============================================================

static const char* role_to_string(ble_conn_role_t role) {
    switch (role) {
        case BLE_CONN_ROLE_PRIMARY: return "PRIMARY";
        case BLE_CONN_ROLE_DEBUG: return "DEBUG";
        case BLE_CONN_ROLE_NONE: return "NONE";
        default: return "UNKNOWN";
    }
}

static int find_empty_slot(void) {
    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (!g_connections[i].is_connected) {
            return i;
        }
    }
    return -1;
}

static int find_slot_by_handle(uint16_t conn_handle) {
    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected && g_connections[i].conn_handle == conn_handle) {
            return i;
        }
    }
    return -1;
}

static bool has_primary(void) {
    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected && g_connections[i].role == BLE_CONN_ROLE_PRIMARY) {
            return true;
        }
    }
    return false;
}

static bool has_debug(void) {
    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected && g_connections[i].role == BLE_CONN_ROLE_DEBUG) {
            return true;
        }
    }
    return false;
}

// ============================================================
// API Implementation
// ============================================================

esp_err_t ble_conn_manager_init(void) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Initialize all connection slots
    memset(g_connections, 0, sizeof(g_connections));
    g_connection_count = 0;
    g_initialized = true;

    ESP_LOGI(TAG, "Initialized (max_connections=%d)", BLE_MAX_CONNECTIONS);
    return ESP_OK;
}

ble_conn_role_t ble_conn_manager_add_connection(uint16_t conn_handle,
                                                const uint8_t *addr,
                                                uint8_t addr_type) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return BLE_CONN_ROLE_NONE;
    }

    // Check if already at max connections
    if (g_connection_count >= BLE_MAX_CONNECTIONS) {
        ESP_LOGE(TAG, "Max connections reached (%d)", BLE_MAX_CONNECTIONS);
        return BLE_CONN_ROLE_NONE;
    }

    // Find empty slot
    int slot = find_empty_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No available slot (should not happen)");
        return BLE_CONN_ROLE_NONE;
    }

    ble_conn_role_t assigned_role;

    // Role assignment logic with persistence
    if (!has_primary()) {
        // Case 1: No PRIMARY exists → assign PRIMARY
        assigned_role = BLE_CONN_ROLE_PRIMARY;
        ESP_LOGI(TAG, "Conn %d assigned as PRIMARY (first primary)", conn_handle);

    } else if (!has_debug()) {
        // Case 2: PRIMARY exists but no DEBUG → assign DEBUG
        assigned_role = BLE_CONN_ROLE_DEBUG;
        ESP_LOGI(TAG, "Conn %d assigned as DEBUG (first debug)", conn_handle);

    } else {
        // Case 3: Both PRIMARY and DEBUG exist → reuse disconnected role
        // Find disconnected slot and reuse its role
        assigned_role = g_connections[slot].role;

        if (assigned_role == BLE_CONN_ROLE_NONE) {
            // Fallback: should not happen, assign PRIMARY
            assigned_role = BLE_CONN_ROLE_PRIMARY;
            ESP_LOGW(TAG, "Conn %d: no role in slot %d, defaulting to PRIMARY",
                     conn_handle, slot);
        } else {
            ESP_LOGI(TAG, "Conn %d reassigned as %s (reusing slot %d)",
                     conn_handle, role_to_string(assigned_role), slot);
        }
    }

    // Fill connection state
    g_connections[slot].conn_handle = conn_handle;
    g_connections[slot].role = assigned_role;
    g_connections[slot].is_connected = true;
    g_connections[slot].addr_type = addr_type;
    memcpy(g_connections[slot].addr, addr, 6);

    g_connection_count++;

    ESP_LOGI(TAG, "Connection added: handle=%d role=%s addr=%02x:%02x:%02x:%02x:%02x:%02x count=%d/%d",
             conn_handle,
             role_to_string(assigned_role),
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
             g_connection_count, BLE_MAX_CONNECTIONS);

    return assigned_role;
}

esp_err_t ble_conn_manager_remove_connection(uint16_t conn_handle) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int slot = find_slot_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "Connection handle %d not found", conn_handle);
        return ESP_ERR_NOT_FOUND;
    }

    ble_conn_role_t removed_role = g_connections[slot].role;

    ESP_LOGI(TAG, "Connection removed: handle=%d role=%s addr=%02x:%02x:%02x:%02x:%02x:%02x",
             conn_handle,
             role_to_string(removed_role),
             g_connections[slot].addr[0],
             g_connections[slot].addr[1],
             g_connections[slot].addr[2],
             g_connections[slot].addr[3],
             g_connections[slot].addr[4],
             g_connections[slot].addr[5]);

    // Mark as disconnected but preserve role for reassignment
    g_connections[slot].is_connected = false;
    g_connections[slot].conn_handle = 0;
    g_connection_count--;

    // Log about role persistence
    if (removed_role == BLE_CONN_ROLE_DEBUG) {
        ESP_LOGI(TAG, "DEBUG disconnected, PRIMARY keeps PRIMARY role");
    } else if (removed_role == BLE_CONN_ROLE_PRIMARY) {
        if (has_debug()) {
            ESP_LOGI(TAG, "PRIMARY disconnected, DEBUG keeps DEBUG role");
        } else {
            ESP_LOGI(TAG, "PRIMARY disconnected, no other connections");
        }
    }

    return ESP_OK;
}

ble_conn_role_t ble_conn_manager_get_role(uint16_t conn_handle) {
    if (!g_initialized) {
        return BLE_CONN_ROLE_NONE;
    }

    int slot = find_slot_by_handle(conn_handle);
    if (slot < 0) {
        return BLE_CONN_ROLE_NONE;
    }

    return g_connections[slot].role;
}

bool ble_conn_manager_is_primary(uint16_t conn_handle) {
    return ble_conn_manager_get_role(conn_handle) == BLE_CONN_ROLE_PRIMARY;
}

bool ble_conn_manager_is_debug(uint16_t conn_handle) {
    return ble_conn_manager_get_role(conn_handle) == BLE_CONN_ROLE_DEBUG;
}

int ble_conn_manager_get_connection_count(void) {
    if (!g_initialized) {
        return 0;
    }
    return g_connection_count;
}

bool ble_conn_manager_is_max_connections(void) {
    return g_connection_count >= BLE_MAX_CONNECTIONS;
}

uint16_t ble_conn_manager_get_primary_handle(void) {
    if (!g_initialized) {
        return 0;
    }

    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected && g_connections[i].role == BLE_CONN_ROLE_PRIMARY) {
            return g_connections[i].conn_handle;
        }
    }

    return 0;
}

uint16_t ble_conn_manager_get_debug_handle(void) {
    if (!g_initialized) {
        return 0;
    }

    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected && g_connections[i].role == BLE_CONN_ROLE_DEBUG) {
            return g_connections[i].conn_handle;
        }
    }

    return 0;
}

bool ble_conn_manager_has_debug(void) {
    return has_debug();
}

esp_err_t ble_conn_manager_get_state(uint16_t conn_handle, ble_conn_state_t *state) {
    if (!g_initialized || !state) {
        return ESP_ERR_INVALID_ARG;
    }

    int slot = find_slot_by_handle(conn_handle);
    if (slot < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &g_connections[slot], sizeof(ble_conn_state_t));
    return ESP_OK;
}

void ble_conn_manager_print_info(void) {
    if (!g_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return;
    }

    ESP_LOGI(TAG, "=== Connection Manager State ===");
    ESP_LOGI(TAG, "Max connections: %d", BLE_MAX_CONNECTIONS);
    ESP_LOGI(TAG, "Active connections: %d", g_connection_count);

    for (int i = 0; i < BLE_MAX_CONNECTIONS; i++) {
        if (g_connections[i].is_connected) {
            ESP_LOGI(TAG, "  [%d] handle=%d role=%s addr=%02x:%02x:%02x:%02x:%02x:%02x",
                     i,
                     g_connections[i].conn_handle,
                     role_to_string(g_connections[i].role),
                     g_connections[i].addr[0],
                     g_connections[i].addr[1],
                     g_connections[i].addr[2],
                     g_connections[i].addr[3],
                     g_connections[i].addr[4],
                     g_connections[i].addr[5]);
        } else {
            ESP_LOGI(TAG, "  [%d] empty (last_role=%s)",
                     i, role_to_string(g_connections[i].role));
        }
    }

    ESP_LOGI(TAG, "Primary handle: %d", ble_conn_manager_get_primary_handle());
    ESP_LOGI(TAG, "Debug handle: %d", ble_conn_manager_get_debug_handle());
    ESP_LOGI(TAG, "===============================");
}
