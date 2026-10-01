# NavNode State Machine

```mermaid
flowchart TD
    A["sm_timer_callback"] --> B["next_state = tc_next_state"]

    B --> C{"tc_state != next_state ?"}
    C -- Yes --> D["stateChange = True"]
    D --> E["log transition"]
    E --> F["tts: Nav timer"]
    C -- No --> G["stateChange = False"]

    G --> H["state = next_state"]
    H --> I["tc_state = state"]

    I --> S1{"state == T_INIT_WAIT"}
    S1 -- Yes --> S1A["tts: wait for nav 2"]
    S1A --> S1B["sleep 20s"]
    S1B --> S1C{"GPS or compass enabled?"}
    S1C -- Yes --> S1D["next_state = T_CAL_IMU"]
    S1C -- No --> S1E["next_state = T_WAIT_GO"]

    I --> S2{"state == T_CAL_IMU"}
    S2 -- Yes --> S2A["status = calImu()"]
    S2A --> S2B{"status == True ?"}
    S2B -- Yes --> S2C["next_state = T_WAIT_GO"]
    S2B -- No --> S2D["stay in T_CAL_IMU"]

    I --> S3{"state == T_WAIT_GO"}
    S3 -- Yes --> S3A{"waitGo() ?"}
    S3A -- Yes --> S3B["next_state = T_SETUP_NAV"]
    S3A -- No --> S3C["stay in T_WAIT_GO"]

    I --> S4{"state == T_SETUP_NAV"}
    S4 -- Yes --> S4A{"setupNav(stateChange) ?"}
    S4A -- Yes --> S4B["next_state = T_GET_WP"]
    S4A -- No --> S4C["stay in T_SETUP_NAV"]

    I --> S5{"state == T_GET_WP"}
    S5 -- Yes --> S5A{"getNextWaypoint() ?"}
    S5A -- Yes --> S5B["next_state = T_NAV_WP"]
    S5A -- No --> S5C["next_state = T_DONE"]

    I --> S6{"state == T_NAV_WP_AGAIN"}
    S6 -- Yes --> S6A["next_state = T_NAV_WP"]

    I --> S7{"state == T_NAV_WP"}
    S7 -- Yes --> S7A{"stateChange ?"}
    S7A -- Yes --> S7B["log waypoint"]
    S7B --> S7C["read waypoint x, y"]
    S7C --> S7D["smTimerNav2Config(T_NAV_WP)"]
    S7D --> S7E["nav.goToPose()"]
    S7E --> S7F{"cone detected while navigating?"}
    S7F -- Yes --> S7G["cancelNav2Task()"]
    S7G --> S7H["next_state = T_GOTO_CONE"]
    S7F -- No --> S7I{"nav.isTaskComplete() ?"}
    S7I -- Yes --> S7J{"result == SUCCEEDED ?"}
    S7J -- Yes --> S7K{"wpCone == True ?"}
    S7K -- Yes --> S7L["next_state = T_GOTO_CONE"]
    S7K -- No --> S7M["next_state = T_GET_WP"]
    S7J -- No --> S7N["next_state = T_NAV_WP_AGAIN"]
    S7I -- No --> S7P["stay in T_NAV_WP"]

    I --> S8{"state == T_GOTO_CONE"}
    S8 -- Yes --> S8A{"stateChange ?"}
    S8A -- Yes --> S8B["tts: Go to the cone"]
    S8B --> S8C["smTimerNav2Config(T_GOTO_CONE)"]
    S8C --> S8D["done = cd_sm()"]
    S8D --> S8E{"done == True ?"}
    S8E -- Yes --> S8F["next_state = T_GET_WP"]
    S8E -- No --> S8G["stay in T_GOTO_CONE"]

    I --> S9{"state == T_DONE"}
    S9 -- Yes --> S9A{"stateChange ?"}
    S9A -- Yes --> S9B["tts: Finished"]
    S9B --> S9C["end mission"]

    Z["tc_next_state = next_state"] --> END["End of callback"]
```

This diagram is based on the transition logic inside `sm_timer_callback` in [src/robocolumbus_stuff/robocolumbus_stuff/nav_node.py](src/robocolumbus_stuff/robocolumbus_stuff/nav_node.py).
