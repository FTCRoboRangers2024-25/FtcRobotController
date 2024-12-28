package org.firstinspires.ftc.teamcode.utilities;

import java.util.HashMap;

public abstract class Component {
    protected HashMap<String, Object> dependencies = new HashMap<>();

    public void addDependencies(HashMap<String, Object> dependencies) {
        this.dependencies.putAll(dependencies);
    }

    public void addDependency(String key, Object value) {
        dependencies.put(key, value);
    }

    public abstract void init();
    public abstract void loop();

    protected <T> T getDependency(String key, Class<T> targetClass) {
        return targetClass.cast(dependencies.get(key));
    }

    protected void broadcastMessage(String message) {
        if (!message.isEmpty()) {
            getDependency("MessageBroadcaster", IMessageBroadcaster.class).broadcastMessage(message);
        }
    }
}
