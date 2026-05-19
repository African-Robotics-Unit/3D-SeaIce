function logInfo(message)

    timestamp = string(datetime("now", ...
        "Format", "yyyy-MM-dd HH:mm:ss.SSS"));

    fprintf("%s | INFO | %s\n", timestamp, message);

end