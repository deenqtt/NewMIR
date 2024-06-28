using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class NewRobot : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "DomainId",
                table: "Robots");

            migrationBuilder.RenameColumn(
                name: "Serialnumber",
                table: "Robots",
                newName: "Port");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.RenameColumn(
                name: "Port",
                table: "Robots",
                newName: "Serialnumber");

            migrationBuilder.AddColumn<string>(
                name: "DomainId",
                table: "Robots",
                type: "TEXT",
                nullable: true);
        }
    }
}
