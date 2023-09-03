#include "Prompter.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

UPS::Prompter::Prompter(std::string script_file) : script_file(script_file)
{

    prompt_pid = fork();

    if (prompt_pid == -1) {
        std::cerr << "[couldn't fork to prompt]" << std::endl;
        assert(false);
    }

    if (prompt_pid == 0) {
        system("touch /tmp/prompt.txt");
        system("xterm -fg white -bg black -fa 'Monospace' -fs 25 -e \"watch -n 0.5 cat /tmp/prompt.txt\"");
        //execlp("xterm", "xterm_ups_prompt", "-e ", "\"watch", "-n","1","cat","/tmp/prompt.txt\"",NULL);
        abort();
    }
}

void UPS::Prompter::write(promptTag tag,TimeStamp fromBegin) const
{
    if (!scripts.contains(tag)){
        //std::cerr << "[INVALID TAG] " << tag << std::endl;
        erase(fromBegin);
        return;
    }
    std::ofstream file("/tmp/prompt.txt");
    file << "|----------------------------------------------------------------|" << std::endl;
    file << "                    UPS PROMPT : TIME "<<std::to_string(std::chrono::duration_cast<std::chrono::minutes>(Time::now()-fromBegin).count()) << " Mns                               " << std::endl;
    file << "|----------------------------------------------------------------|" << std::endl;
    file << scripts.at(tag);
    file.close();
}

void UPS::Prompter::loadScript()
{
    std::ifstream script(script_file);
    std::string line;
    while (std::getline(script,line))
    {
        if (current_tag.empty() && line.empty())
            continue;
        if (line[0] == '['){
            line.erase(remove(line.begin(), line.end(), '['), line.end());
            line.erase(remove(line.begin(), line.end(), ']'), line.end());
            current_tag = line;
        }
        else if (!line.empty() && current_tag.empty()) {
            std::cerr << " [INVALID SCRIPT FORMAT, MUST INSERT \"[TAG]\" BEFORE TEXT ]" << std::endl;
        }
        else {
            if (!scripts.contains(current_tag))
                scripts[current_tag] = "";
            scripts[current_tag] += line + "\n";
        }
    }
}

void UPS::Prompter::erase(TimeStamp fromBegin) const
{
    std::ofstream file("/tmp/prompt.txt");
    file << "|----------------------------------------------------------------|" << std::endl;
    file << "                    UPS PROMPT : TIME "<<std::to_string(std::chrono::duration_cast<std::chrono::minutes>(Time::now()-fromBegin).count()) << " Mns                               " << std::endl;
    file << "|----------------------------------------------------------------|" << std::endl;
    file.close();
    return;
}
